#pragma once

#include <ArduinoJson.h>

namespace services::net {

/** Hook invoked while waiting on the network (button poll, watchdog feed). */
using PollFn = void (*)();

/**
 * HTTPS GET `url` and deserialize the body straight from the TLS stream into
 * `doc`, keeping only the fields selected by `filter`. Nothing is buffered in
 * a String first, so peak heap is the TLS session plus the (filtered)
 * document — on the C3, with the 115 KB frame sprite resident, that is the
 * difference between a heap that stays healthy for days and one that
 * fragments until TLS can no longer allocate its record buffer.
 *
 * `*http_code` gets the status code (<= 0 on transport failure). The body is
 * parsed for 200, and also for 404 when `accept_not_found` is set (hexdb /
 * adsbdb answer a miss with 404 + JSON). Returns true when a body was parsed.
 */
bool getJson(const char* url, JsonVariantConst filter, JsonDocument& doc,
             PollFn poll, bool accept_not_found, int* http_code);

}  // namespace services::net
