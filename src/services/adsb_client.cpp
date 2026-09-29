#include "services/adsb_client.h"

#include <Arduino.h>
#include <ArduinoJson.h>

#include <cstring>

#include "config.h"
#include "services/flight_route.h"
#include "services/https_json.h"
#include "services/track_history.h"
#include "ui/radar_range.h"

namespace services::adsb {

namespace {

constexpr char kApiBase[] = "https://opendata.adsb.fi/api/v3/lat/";
constexpr float kKmPerNm = 1.852f;

Aircraft s_aircraft[kMaxAircraft];
size_t s_aircraft_count = 0;
PollFn s_poll_fn = nullptr;

// Only the fields read below survive parsing; the rest of each adsb.fi record
// (mlat, nac_p, rssi, squawk, ...) is dropped on the fly, cutting the document
// several-fold.
JsonVariantConst aircraftFilter() {
  static JsonDocument filter;
  if (filter.isNull()) {
    JsonObject plane = filter["ac"].add<JsonObject>();
    for (const char* key :
         {"hex", "flight", "t", "lat", "lon", "alt_baro", "alt_geom",
          "true_heading", "mag_heading", "track", "dir", "gs", "tas", "ias",
          "baro_rate", "geom_rate"}) {
      plane[key] = true;
    }
  }
  return filter.as<JsonVariantConst>();
}

float kmToNauticalMiles(float km) { return km / kKmPerNm; }

bool readJsonFloat(const JsonObject& obj, const char* key, float* out) {
  if (obj[key].is<float>() || obj[key].is<double>() || obj[key].is<int>()) {
    *out = obj[key].as<float>();
    return true;
  }
  return false;
}

float pickNoseHeading(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "true_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "mag_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "track", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "dir", &v)) {
    return v;
  }
  return 0.0f;
}

float pickTrackHeading(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "track", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "true_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "mag_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "dir", &v)) {
    return v;
  }
  return 0.0f;
}

float pickGroundSpeed(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "gs", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "tas", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "ias", &v)) {
    return v;
  }
  return 0.0f;
}

float pickVerticalRate(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "baro_rate", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "geom_rate", &v)) {
    return v;
  }
  return 0.0f;
}

bool isOnGround(const JsonObject& plane) {
  if (!plane["alt_baro"].is<const char*>()) {
    return false;
  }
  return strcmp(plane["alt_baro"].as<const char*>(), "ground") == 0;
}

void copyJsonStringTrimmed(const JsonObject& obj, const char* key, char* out,
                           size_t out_len) {
  out[0] = '\0';
  if (out_len == 0 || !obj[key].is<const char*>()) {
    return;
  }
  const char* s = obj[key].as<const char*>();
  size_t n = strnlen(s, out_len - 1);
  while (n > 0 && s[n - 1] == ' ') {
    --n;
  }
  memcpy(out, s, n);
  out[n] = '\0';
}

void formatAltitudeTag(const JsonObject& plane, char* out, size_t out_len) {
  out[0] = '\0';
  if (out_len == 0) {
    return;
  }

  if (plane["alt_baro"].is<const char*>()) {
    const char* s = plane["alt_baro"].as<const char*>();
    if (strcmp(s, "ground") == 0) {
      strncpy(out, "GND", out_len - 1);
      out[out_len - 1] = '\0';
      return;
    }
  }

  float alt = 0.0f;
  if (readJsonFloat(plane, "alt_baro", &alt) ||
      readJsonFloat(plane, "alt_geom", &alt)) {
    snprintf(out, out_len, "%d ft", static_cast<int>(lroundf(alt)));
  }
}

void fillTagFields(Aircraft* ac, const JsonObject& plane) {
  copyJsonStringTrimmed(plane, "hex", ac->hex, sizeof(ac->hex));
  copyJsonStringTrimmed(plane, "flight", ac->callsign, sizeof(ac->callsign));
  if (ac->callsign[0] == '\0') {
    copyJsonStringTrimmed(plane, "hex", ac->callsign, sizeof(ac->callsign));
  }

  copyJsonStringTrimmed(plane, "t", ac->type, sizeof(ac->type));
  formatAltitudeTag(plane, ac->alt, sizeof(ac->alt));
  ac->airline[0] = '\0';
  ac->origin[0] = '\0';
  ac->dest[0] = '\0';
}

}  // namespace

void setPollFn(PollFn fn) { s_poll_fn = fn; }

size_t aircraftCount() { return s_aircraft_count; }

const Aircraft* aircraftList() { return s_aircraft; }

// Cached answers are free; unresolved callsigns cost up to two HTTPS GETs each
// (hexdb route + adsbdb airline) and are capped per cycle so the ADS-B poll
// interval is not blown. Runs from the main loop *after* fetchUpdate() so only
// one WiFiClientSecure is alive at a time.
void resolveRoutes() {
  if (!config::kRouteLookupEnabled) {
    return;
  }
  uint8_t budget = config::kRouteLookupsPerCycle;
  for (size_t i = 0; i < s_aircraft_count; ++i) {
    Aircraft& ac = s_aircraft[i];
    const route::Result r = route::resolve(
        ac.callsign, ac.origin, sizeof(ac.origin), ac.dest, sizeof(ac.dest),
        ac.airline, sizeof(ac.airline), ac.lat, ac.lon,
        ui::radar::routeFullNames(), s_poll_fn, budget > 0);
    if (r == route::Result::kFetched && budget > 0) {
      --budget;
    }
  }
}

bool fetchUpdate(double center_lat, double center_lon, float fetch_radius_km) {
  const float dist_nm = kmToNauticalMiles(fetch_radius_km);

  String url = kApiBase;
  url += String(center_lat, 6);
  url += "/lon/";
  url += String(center_lon, 6);
  url += "/dist/";
  url += String(dist_nm, 1);

  JsonDocument doc;
  int code = 0;
  if (!net::getJson(url.c_str(), aircraftFilter(), doc, s_poll_fn,
                    /*accept_not_found=*/false, &code)) {
    Serial.printf("adsb: fetch failed (http %d)\n", code);
    return false;
  }

  JsonArray ac = doc["ac"].as<JsonArray>();
  if (ac.isNull()) {
    s_aircraft_count = 0;
    return true;
  }

  size_t n = 0;
  for (JsonObject plane : ac) {
    if (n >= kMaxAircraft) {
      break;
    }
    if (!plane["lat"].is<float>() || !plane["lon"].is<float>()) {
      continue;
    }
    if (isOnGround(plane) && !config::kAdsbShowGroundAircraft) {
      continue;
    }

    s_aircraft[n].lat = plane["lat"].as<float>();
    s_aircraft[n].lon = plane["lon"].as<float>();
    s_aircraft[n].nose_deg = pickNoseHeading(plane);
    s_aircraft[n].track_deg = pickTrackHeading(plane);
    s_aircraft[n].gs_knots = pickGroundSpeed(plane);
    s_aircraft[n].vert_rate_fpm = pickVerticalRate(plane);
    fillTagFields(&s_aircraft[n], plane);
    ++n;
  }

  s_aircraft_count = n;

  for (size_t i = 0; i < n; ++i) {
    track::record(s_aircraft[i].hex, s_aircraft[i].lat, s_aircraft[i].lon);
  }
  track::expireStale();

  Serial.printf("adsb: %u aircraft (heap %u, max block %u)\n",
                static_cast<unsigned>(n), ESP.getFreeHeap(),
                ESP.getMaxAllocHeap());
  return true;
}

}  // namespace services::adsb
