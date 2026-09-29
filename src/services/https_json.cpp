#include "services/https_json.h"

#include <HTTPClient.h>
#include <WiFiClientSecure.h>

namespace services::net {

namespace {

/** TCP connect. The old 200 ms value was shorter than a typical internet RTT
 *  + SYN/ACK, so most connects failed and were retried in a tight loop — each
 *  retry a full mbedTLS context alloc/free, which fragments the heap. */
constexpr int kConnectTimeoutMs = 5000;
/** TLS handshake (core default is 120 s). */
constexpr unsigned long kHandshakeTimeoutSec = 10;
/** Per-read socket timeout. */
constexpr uint16_t kReadTimeoutMs = 8000;
/** Hard cap on reading + parsing the body, however slowly it trickles in. */
constexpr unsigned long kBodyDeadlineMs = 15000;

void callPoll(PollFn poll) {
  if (poll != nullptr) {
    poll();
  }
}

// ArduinoJson "custom reader": serves the parser from a small local buffer
// refilled with whatever the socket has ready. Reading only available() bytes
// (rather than Stream::readBytes, which waits for a full buffer) means the
// tail of an HTTP/1.0 body — delimited by connection close — does not stall
// for a whole read timeout.
class BodyReader {
 public:
  BodyReader(WiFiClient& client, PollFn poll)
      : client_(client), poll_(poll), deadline_(millis() + kBodyDeadlineMs) {}

  int read() {
    if (pos_ == len_ && !fill()) {
      return -1;
    }
    return buf_[pos_++];
  }

  size_t readBytes(char* out, size_t n) {
    size_t done = 0;
    while (done < n) {
      if (pos_ == len_ && !fill()) {
        break;
      }
      size_t chunk = len_ - pos_;
      if (chunk > n - done) {
        chunk = n - done;
      }
      memcpy(out + done, buf_ + pos_, chunk);
      pos_ += chunk;
      done += chunk;
    }
    return done;
  }

 private:
  bool fill() {
    pos_ = len_ = 0;
    while (static_cast<long>(millis() - deadline_) < 0) {
      callPoll(poll_);
      const int avail = client_.available();
      if (avail > 0) {
        const size_t want =
            static_cast<size_t>(avail) < sizeof(buf_) ? avail : sizeof(buf_);
        const int got = client_.read(buf_, want);
        if (got > 0) {
          len_ = static_cast<size_t>(got);
          return true;
        }
      } else if (!client_.connected()) {
        return false;
      }
      delay(2);
    }
    return false;
  }

  WiFiClient& client_;
  PollFn poll_;
  unsigned long deadline_;
  uint8_t buf_[256];
  size_t pos_ = 0;
  size_t len_ = 0;
};

}  // namespace

bool getJson(const char* url, JsonVariantConst filter, JsonDocument& doc,
             PollFn poll, bool accept_not_found, int* http_code) {
  *http_code = 0;
  doc.clear();

  // Declared before `http` so the HTTPClient (which references it) goes first.
  WiFiClientSecure client;
  client.setInsecure();
  client.setHandshakeTimeout(kHandshakeTimeoutSec);

  HTTPClient http;
  if (!http.begin(client, url)) {
    return false;
  }
  // The APIs sit behind CDNs that chunk HTTP/1.1 replies; HTTP/1.0 makes the
  // body arrive unframed, delimited by connection close.
  http.useHTTP10(true);
  http.setReuse(false);
  http.setConnectTimeout(kConnectTimeoutMs);
  http.setTimeout(kReadTimeoutMs);

  callPoll(poll);
  const int code = http.GET();
  *http_code = code;
  const bool want_body =
      code == HTTP_CODE_OK || (accept_not_found && code == HTTP_CODE_NOT_FOUND);
  WiFiClient* stream = want_body ? http.getStreamPtr() : nullptr;
  if (stream == nullptr) {
    http.end();
    return false;
  }

  BodyReader reader(*stream, poll);
  const DeserializationError err =
      deserializeJson(doc, reader, DeserializationOption::Filter(filter));
  http.end();
  if (err) {
    Serial.printf("net: JSON %s (http %d)\n", err.c_str(), code);
    return false;
  }
  return true;
}

}  // namespace services::net
