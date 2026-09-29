/**
 * Plane Radar — WiFi setup, then radar UI on the round GC9A01 display.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "hardware/display.h"
#include "services/adsb_client.h"
#include "services/radar_location.h"
#include "services/wifi_setup.h"
#include "ui/radar_display.h"
#include "ui/radar_range.h"
#include "ui/status_screens.h"

namespace {

bool g_radar_visible = false;
unsigned long g_wifi_down_since = 0;
unsigned long g_last_reconnect_ms = 0;
unsigned long g_last_adsb_fetch_ms = 0;
unsigned long g_last_adsb_ok_ms = 0;
uint8_t g_fetch_fails = 0;
bool g_stale_shown = false;

// Runs while an HTTPS request blocks: keep the watchdog fed and the long-press
// reset responsive (taps are latched by the ISR and handled afterwards).
// Deliberately does *not* service the WiFiManager web server — handling a
// portal request (page build, WiFi scan) in the middle of a TLS session
// competes for the same scarce heap.
void pollDuringIo() {
  esp_task_wdt_reset();
  bootButtonPollLongPress();
}

void startLoopWatchdog() {
  // The core already runs the task WDT (idle task, 5 s); re-init only
  // updates its timeout and makes it panic -> reboot.
  esp_task_wdt_init(config::kLoopWatchdogSec, true);
  esp_task_wdt_add(nullptr);
}

void rebootNow(const char* why) {
  Serial.printf("Rebooting: %s\n", why);
  Serial.flush();
  delay(200);
  esp_restart();
}

// Called after every fetch attempt: escalate from a WiFi bounce to a reboot
// when the feed stays dead, and reboot early if the heap is too fragmented
// for TLS to ever succeed again.
void checkHealth(bool fetch_ok) {
  if (fetch_ok) {
    g_fetch_fails = 0;
    return;
  }
  ++g_fetch_fails;
  const uint32_t max_block = ESP.getMaxAllocHeap();
  Serial.printf("adsb: %u consecutive failures (heap %u, max block %u)\n",
                g_fetch_fails, ESP.getFreeHeap(), max_block);
  if (max_block < config::kRebootMinMaxAllocHeap) {
    rebootNow("heap too fragmented for TLS");
  }
  if (g_fetch_fails >= config::kFetchFailsReboot) {
    rebootNow("ADS-B feed dead");
  }
  if (g_fetch_fails == config::kFetchFailsWifiReset) {
    Serial.println("adsb: bouncing WiFi");
    WiFi.disconnect();  // loop() sees the link down and reconnects
  }
}

/** Sync the "NO DATA" badge with the feed age; true when it changed (the
 *  caller then owes a redraw). */
bool updateStaleBadge() {
  const bool stale = millis() - g_last_adsb_ok_ms >= config::kAdsbStaleMs;
  if (stale == g_stale_shown) {
    return false;
  }
  g_stale_shown = stale;
  ui::radarDisplaySetStale(stale);
  return true;
}

void showRadarIfConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    g_radar_visible = false;
    return;
  }
  ui::radarDisplayDraw();
  g_radar_visible = true;
}

void onRangeTap() {
  ui::radar::rangeNext();
  char range_label[12];
  ui::radar::formatCurrentRing3Label(range_label, sizeof(range_label));
  Serial.printf("Range: %s (outer ~%.0f km)\n", range_label,
                ui::radar::rangeCurrent().outer_km);

  if (g_radar_visible && WiFi.status() == WL_CONNECTED) {
    ui::radarDisplayDraw();
  }
}

void handleBootButton() {
  bootButtonPollLongPress();
  if (bootButtonConsumeTap()) {
    onRangeTap();
  }
}

void fetchAndDrawAircraft() {
  const float fetch_km = ui::radar::fetchRadiusKm();
  const bool ok = services::adsb::fetchUpdate(services::location::lat(),
                                              services::location::lon(), fetch_km);
  checkHealth(ok);
  if (!ok) {
    handleBootButton();
    return;
  }
  g_last_adsb_ok_ms = millis();
  // Separate HTTPS call, run only after fetchUpdate()'s TLS session is gone.
  // Skipped while the heap is tight so the extra TLS sessions cannot starve
  // the next ADS-B fetch.
  if (ESP.getMaxAllocHeap() >= config::kRouteMinMaxAllocHeap) {
    services::adsb::resolveRoutes();
  }
  updateStaleBadge();
  ui::radarDisplayRefreshAircraft();
  handleBootButton();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("Plane Radar");

  bootButtonInit();
  displayInit();
  if (wifiShowsSetupScreenOnBoot()) {
    statusScreenPortal();
  }
  services::location::init();
  ui::radar::rangeInit();
  services::adsb::setPollFn(pollDuringIo);

  if (wifiSetupConnect()) {
    showRadarIfConnected();
  }
  // Armed after setup: the first-run config portal may legitimately block
  // for as long as the user takes to fill it in.
  g_last_adsb_ok_ms = millis();
  startLoopWatchdog();
}

void loop() {
  esp_task_wdt_reset();
  handleBootButton();
  wifiLoop();

  if (WiFi.status() != WL_CONNECTED) {
    if (g_radar_visible) {
      Serial.println("WiFi lost — will reconnect");
      g_radar_visible = false;
    }

    if (g_wifi_down_since == 0) {
      g_wifi_down_since = millis();
    }

    const unsigned long down_ms = millis() - g_wifi_down_since;
    if (down_ms >= config::kWifiDownGraceMs &&
        millis() - g_last_reconnect_ms >= config::kWifiReconnectIntervalMs) {
      g_last_reconnect_ms = millis();
      if (wifiReconnect()) {
        g_wifi_down_since = 0;
        showRadarIfConnected();
      }
    }
  } else {
    g_wifi_down_since = 0;
    if (!g_radar_visible) {
      showRadarIfConnected();
    } else if (millis() - g_last_adsb_fetch_ms >= config::kAdsbFetchIntervalMs) {
      g_last_adsb_fetch_ms = millis();
      fetchAndDrawAircraft();
    } else if (updateStaleBadge()) {
      ui::radarDisplayRefreshAircraft();
    } else {
      // Between fetches: let stacked aircraft tags take turns.
      ui::radarDisplayAnimTick();
    }
  }

  delay(10);
}
