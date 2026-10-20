// Lumi combined instrument diagnostic v1.1.8 for LILYGO T-RSS3 / ESP32-S3.
// Read-only inputs: Navman S100 through onboard RS232 and NASA Wind on GPIO18.
// Wi-Fi forwards valid NMEA0183 sentences and provides OTA plus diagnostics.

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>

class NmeaReceiver;
class NmeaUdpForwarder;

#if !CONFIG_IDF_TARGET_ESP32S3
#error "This sketch is intended for the LILYGO T-RSS3 / ESP32-S3."
#endif

constexpr char FIRMWARE_VERSION[] = "1.1.8";
constexpr char WIFI_SSID[] = "lumi";
constexpr char WIFI_PASSWORD[] = "sonich123";
constexpr char OTA_HOSTNAME[] = "lumi-instrument";

constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t NMEA_BAUD = 4800;
constexpr int NAVMAN_RX_PIN = 42;
constexpr int NAVMAN_TX_PIN = 41;  // Assigned by the board; this sketch never writes to it.
constexpr int NASA_RX_PIN = 18;
const IPAddress SIGNALK_HOST(10, 42, 0, 1);
constexpr uint16_t NAVMAN_UDP_PORT = 10111;
constexpr uint16_t NASA_UDP_PORT = 10112;
constexpr uint32_t PARTIAL_TIMEOUT_MS = 1000;
constexpr uint32_t STATUS_INTERVAL_MS = 10000;
constexpr size_t LINE_CAPACITY = 160;
constexpr size_t RAW_CAPACITY = 64;
constexpr uint32_t LINE_TIMEOUT_MS = 2000;

HardwareSerial nasaSerial(1);
WebServer diagnosticServer(80);
bool nasaInverted = true;

int hexDigit(char value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  return -1;
}

constexpr uint8_t CHECKSUM_OK = 0;
constexpr uint8_t CHECKSUM_MISSING = 1;
constexpr uint8_t CHECKSUM_INVALID = 2;

class NmeaUdpForwarder {
 public:
  NmeaUdpForwarder(const char *sourceName, uint16_t destinationPort)
      : name(sourceName), port(destinationPort) {}

  void poll() { readyToSend = WiFi.status() == WL_CONNECTED; }

  void sendSentence(const char *line, size_t length) {
    if (WiFi.status() != WL_CONNECTED) {
      readyToSend = false;
      ++droppedSentences;
      return;
    }

    readyToSend = true;
    if (udp.beginPacket(SIGNALK_HOST, port) != 1) {
      ++droppedSentences;
      return;
    }

    const size_t lineBytes = udp.write(
        reinterpret_cast<const uint8_t *>(line), length);
    const size_t endingBytes = udp.write(
        reinterpret_cast<const uint8_t *>("\r\n"), 2);
    const int result = udp.endPacket();
    if (lineBytes == length && endingBytes == 2 && result == 1) {
      ++sentSentences;
    } else {
      ++droppedSentences;
    }
  }

  void printStatus() const {
    Serial.print("[UDP ");
    Serial.print(name);
    Serial.print("] Port ");
    Serial.print(port);
    Serial.print(readyToSend ? ", bereit" : ", wartet auf WLAN");
    Serial.print(", gesendet: ");
    Serial.print(sentSentences);
    Serial.print(", verworfen: ");
    Serial.println(droppedSentences);
  }

  const char *sourceName() const { return name; }
  uint16_t destinationPort() const { return port; }
  bool isReady() const { return readyToSend; }
  uint32_t sentencesSent() const { return sentSentences; }
  uint32_t sentencesDropped() const { return droppedSentences; }

 private:
  const char *name;
  const uint16_t port;
  WiFiUDP udp;
  uint32_t sentSentences = 0;
  uint32_t droppedSentences = 0;
  bool readyToSend = false;
};

uint8_t verifyNmeaChecksum(const char *line, size_t length) {
  if (length < 5 || (line[0] != '$' && line[0] != '!')) {
    return CHECKSUM_INVALID;
  }

  size_t star = 1;
  while (star < length && line[star] != '*') ++star;
  if (star == length) return CHECKSUM_MISSING;
  if (star + 3 != length) return CHECKSUM_INVALID;

  const int high = hexDigit(line[star + 1]);
  const int low = hexDigit(line[star + 2]);
  if (high < 0 || low < 0) return CHECKSUM_INVALID;

  uint8_t calculated = 0;
  for (size_t i = 1; i < star; ++i) {
    calculated ^= static_cast<uint8_t>(line[i]);
  }
  const uint8_t expected = static_cast<uint8_t>((high << 4) | low);
  return calculated == expected ? CHECKSUM_OK : CHECKSUM_INVALID;
}

bool isCompleteUncheckedNmea(const char *line, size_t length) {
  // This is a framing check, not a replacement for a sender checksum.
  if (length < 7 || (line[0] != '$' && line[0] != '!') || line[6] != ',') return false;
  for (size_t i = 1; i < 6; ++i) {
    if (!((line[i] >= 'A' && line[i] <= 'Z') || (line[i] >= '0' && line[i] <= '9'))) return false;
  }
  for (size_t i = 7; i < length; ++i) {
    const uint8_t byte = static_cast<uint8_t>(line[i]);
    if (byte < 32 || byte > 126 || byte == '$' || byte == '!' || byte == '*') return false;
  }
  return true;
}

class NmeaReceiver {
 public:
  NmeaReceiver(const char *sourceName, HardwareSerial &port,
               NmeaUdpForwarder &networkForwarder, bool acceptWithoutChecksum = false)
      : name(sourceName), serialPort(port), forwarder(networkForwarder),
        allowMissingChecksum(acceptWithoutChecksum) {}

  void poll() {
    // Limit each pass so a continuous invalid stream cannot starve OTA or HTTP.
    size_t budget = 256;
    while (budget-- > 0 && serialPort.available() > 0) {
      const int value = serialPort.read();
      if (value < 0) break;

      const uint8_t byte = static_cast<uint8_t>(value);
      ++receivedBytes;
      lastByteAt = millis();
      rawBytes[rawNext] = byte;
      rawNext = (rawNext + 1) % RAW_CAPACITY;
      if (rawCount < RAW_CAPACITY) ++rawCount;
      if (byte == 0x00) ++zeroBytes;
      if (byte == 0xFF) ++ffBytes;
      if (byte == '\r') ++crBytes;
      if (byte == '\n') ++lfBytes;
      if (byte == '$') ++dollarBytes;

      // The real NASA sender uses CR only; CRLF must still count just once.
      if (byte == '\r' || byte == '\n') {
        printBufferedLine(false);
      } else {
        if (lineLength == 0) lineStartedAt = lastByteAt;
        if (lineLength < LINE_CAPACITY - 1) {
          lineBuffer[lineLength++] = static_cast<char>(byte);
        } else {
          lineOverflow = true;
        }
      }
    }

    const uint32_t now = millis();
    if ((lineLength > 0 || lineOverflow) &&
        (now - lastByteAt >= PARTIAL_TIMEOUT_MS ||
         now - lineStartedAt >= LINE_TIMEOUT_MS)) {
      printBufferedLine(true);
    }
  }

  void printStatus() const {
    Serial.print('[');
    Serial.print(name);
    Serial.print("] Bytes: ");
    Serial.print(receivedBytes);
    Serial.print(", Saetze OK: ");
    Serial.print(validLines);
    Serial.print(", fehlerhaft: ");
    Serial.print(invalidLines);
    Serial.print(", ohne Pruefsumme: ");
    Serial.println(linesWithoutChecksum);
  }

  const char *sourceName() const { return name; }
  const char *latestLine() const { return lastDisplayedLine; }
  uint32_t bytesReceived() const { return receivedBytes; }
  uint32_t validSentenceCount() const { return validLines; }
  uint32_t invalidSentenceCount() const { return invalidLines; }
  uint32_t sentencesWithoutChecksum() const { return linesWithoutChecksum; }
  uint32_t zeroByteCount() const { return zeroBytes; }
  uint32_t ffByteCount() const { return ffBytes; }
  uint32_t crByteCount() const { return crBytes; }
  uint32_t lfByteCount() const { return lfBytes; }
  uint32_t dollarByteCount() const { return dollarBytes; }
  bool forwardsWithoutChecksum() const { return allowMissingChecksum; }

  String rawHex() const {
    String result;
    result.reserve(RAW_CAPACITY * 3);
    const size_t start = (rawNext + RAW_CAPACITY - rawCount) % RAW_CAPACITY;
    for (size_t i = 0; i < rawCount; ++i) {
      char value[4];
      snprintf(value, sizeof(value), "%02X ", rawBytes[(start + i) % RAW_CAPACITY]);
      result += value;
    }
    return result;
  }

  String rawAscii() const {
    String result;
    result.reserve(RAW_CAPACITY);
    const size_t start = (rawNext + RAW_CAPACITY - rawCount) % RAW_CAPACITY;
    for (size_t i = 0; i < rawCount; ++i) {
      const uint8_t byte = rawBytes[(start + i) % RAW_CAPACITY];
      result += (byte >= 32 && byte <= 126) ? static_cast<char>(byte) : '.';
    }
    return result;
  }

  void resetDiagnostics() {
    lineLength = 0;
    lineOverflow = false;
    lastDisplayedLine[0] = '\0';
    rawNext = rawCount = 0;
    receivedBytes = validLines = invalidLines = linesWithoutChecksum = 0;
    zeroBytes = ffBytes = crBytes = lfBytes = dollarBytes = 0;
    lastByteAt = lineStartedAt = millis();
  }

 private:
  void printBufferedLine(bool partial) {
    if (lineLength == 0 && !lineOverflow) return;

    lineBuffer[lineLength] = '\0';
    // Never use raw UART bytes as a C string: a received 00 hides the remainder.
    size_t displayed = 0;
    if (partial) {
      memcpy(lastDisplayedLine, "TEILSTUECK: ", 12);
      displayed = 12;
    }
    for (size_t i = 0; i < lineLength; ++i) {
      const uint8_t byte = static_cast<uint8_t>(lineBuffer[i]);
      lastDisplayedLine[displayed++] = byte >= 32 && byte <= 126 ? static_cast<char>(byte) : '.';
    }
    lastDisplayedLine[displayed] = '\0';
    Serial.print('[');
    Serial.print(name);
    Serial.print(' ');
    Serial.print(millis());
    Serial.print(partial ? " ms] TEILSTUECK: " : " ms] ");

    for (size_t i = 0; i < lineLength; ++i) {
      const uint8_t byte = static_cast<uint8_t>(lineBuffer[i]);
      Serial.print(byte >= 32 && byte <= 126 ? static_cast<char>(byte) : '.');
    }
    Serial.println();

    if (lineOverflow) {
      Serial.println("  WARNUNG: Satzpuffer ueberlaufen");
      ++invalidLines;
    } else if (!partial) {
      const uint8_t result = verifyNmeaChecksum(lineBuffer, lineLength);
      Serial.print("  Pruefsumme: ");
      if (result == CHECKSUM_OK) {
        Serial.println("OK");
        ++validLines;
        forwarder.sendSentence(lineBuffer, lineLength);
      } else if (result == CHECKSUM_MISSING) {
        Serial.println("nicht vorhanden");
        ++linesWithoutChecksum;
        if (allowMissingChecksum) {
          if (isCompleteUncheckedNmea(lineBuffer, lineLength)) {
            forwarder.sendSentence(lineBuffer, lineLength);
          } else {
            ++invalidLines;
            Serial.println("  Verworfen: ungueltiger NMEA-Satzaufbau");
          }
        }
      } else {
        Serial.println("FEHLER");
        ++invalidLines;
      }
    }

    lineLength = 0;
    lineOverflow = false;
  }

  const char *name;
  HardwareSerial &serialPort;
  NmeaUdpForwarder &forwarder;
  const bool allowMissingChecksum;
  char lineBuffer[LINE_CAPACITY]{};
  char lastDisplayedLine[LINE_CAPACITY + 12]{};
  size_t lineLength = 0;
  bool lineOverflow = false;
  uint32_t lastByteAt = 0;
  uint32_t receivedBytes = 0;
  uint32_t validLines = 0;
  uint32_t invalidLines = 0;
  uint32_t linesWithoutChecksum = 0;
  uint8_t rawBytes[RAW_CAPACITY]{};
  size_t rawNext = 0;
  size_t rawCount = 0;
  uint32_t zeroBytes = 0;
  uint32_t ffBytes = 0;
  uint32_t crBytes = 0;
  uint32_t lfBytes = 0;
  uint32_t dollarBytes = 0;
  uint32_t lineStartedAt = 0;
};

NmeaUdpForwarder navmanForwarder("NAVMAN", NAVMAN_UDP_PORT);
NmeaUdpForwarder nasaForwarder("NASA", NASA_UDP_PORT);
NmeaReceiver navmanReceiver("NAVMAN", Serial0, navmanForwarder, true);
NmeaReceiver nasaReceiver("NASA", nasaSerial, nasaForwarder);

volatile int pendingDisconnectReason = -1;
bool networkServicesStarted = false;

void handleWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    pendingDisconnectReason = static_cast<int>(info.wifi_sta_disconnected.reason);
  }
}

void printWiFiStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WLAN] Verbunden, IP: ");
    Serial.print(WiFi.localIP());
    Serial.print(", RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.print("[WLAN] Nicht verbunden, Status: ");
    Serial.println(static_cast<int>(WiFi.status()));
  }
}

void appendHtmlEscaped(String &target, const char *text) {
  while (*text != '\0') {
    switch (*text) {
      case '&': target += F("&amp;"); break;
      case '<': target += F("&lt;"); break;
      case '>': target += F("&gt;"); break;
      case '"': target += F("&quot;"); break;
      default: target += *text; break;
    }
    ++text;
  }
}

void appendReceiverCard(String &html, const NmeaReceiver &receiver) {
  html += F("<section><h2>");
  html += receiver.sourceName();
  html += F("</h2><dl><dt>Bytes</dt><dd>");
  html += receiver.bytesReceived();
  html += F("</dd><dt>Saetze OK</dt><dd>");
  html += receiver.validSentenceCount();
  html += F("</dd><dt>Fehlerhaft</dt><dd>");
  html += receiver.invalidSentenceCount();
  html += F("</dd><dt>Ohne Pruefsumme</dt><dd>");
  html += receiver.sentencesWithoutChecksum();
  html += F("</dd></dl><h3>Letzte Zeile</h3><pre>");
  if (receiver.latestLine()[0] == '\0') {
    html += receiver.bytesReceived() == 0 ? F("Noch keine Bytes empfangen")
                                        : F("Bytes vorhanden; noch keine abgeschlossene Zeile");
  } else {
    appendHtmlEscaped(html, receiver.latestLine());
  }
  html += F("</pre><p>");
  html += receiver.forwardsWithoutChecksum()
              ? F("Vollstaendige Saetze ohne Pruefsumme werden unveraendert weitergeleitet. "
                  "Saetze OK zaehlt nur Saetze mit gueltiger Pruefsumme. "
                  "Signal K: Validate Checksum fuer NAVMAN ausschalten.")
              : F("Weiterleitung nur mit gueltiger Pruefsumme.");
  html += F("</p></section>");
}

void appendNasaDiagnostics(String &html) {
  html += F("<section><h2>NASA Rohdaten / Polaritaet</h2><p>4800 Baud, 8N1 &middot; Empfang: <strong>");
  html += nasaInverted ? F("INVERTIERT") : F("NORMAL");
  html += F("</strong></p><form method='post' action='/nasa-polarity'>"
            "<button name='mode' value='normal'>Normal testen</button> "
            "<button name='mode' value='inverted'>Invertiert testen</button></form>"
            "<p>Beim Wechsel werden nur die NASA-Empfangszaehler und Rohdaten geloescht. "
            "Nach Neustart gilt INVERTIERT. WLAN-Sendezaehler laufen weiter.</p>"
            "<dl><dt>00-Bytes</dt><dd>");
  html += nasaReceiver.zeroByteCount();
  html += F("</dd><dt>FF-Bytes</dt><dd>");
  html += nasaReceiver.ffByteCount();
  html += F("</dd><dt>CR / LF</dt><dd>");
  html += nasaReceiver.crByteCount();
  html += F(" / ");
  html += nasaReceiver.lfByteCount();
  html += F("</dd><dt>$-Zeichen</dt><dd>");
  html += nasaReceiver.dollarByteCount();
  html += F("</dd></dl><h3>Letzte bis zu 64 Bytes (HEX)</h3><pre>");
  html += nasaReceiver.rawHex();
  html += F("</pre><h3>Dieselben Bytes als Text (. = nicht druckbar)</h3><pre>");
  const String ascii = nasaReceiver.rawAscii();
  appendHtmlEscaped(html, ascii.c_str());
  html += F("</pre></section>");
}

void handleNasaPolarity() {
  const String mode = diagnosticServer.arg("mode");
  if (mode != "normal" && mode != "inverted") {
    diagnosticServer.send(400, "text/plain", "Ungueltiger NASA-Modus");
    return;
  }
  const bool inverted = mode == "inverted";
  if (!nasaSerial.setRxInvert(inverted)) {
    diagnosticServer.send(500, "text/plain", "NASA-Empfangspolaritaet konnte nicht geaendert werden");
    return;
  }
  // Drop bytes already buffered under the previous polarity.
  for (size_t i = 0; i < 2048 && nasaSerial.available() > 0; ++i) nasaSerial.read();
  nasaInverted = inverted;
  nasaReceiver.resetDiagnostics();
  Serial.printf("[NASA] Empfang: %s; Diagnose zurueckgesetzt\n", inverted ? "INVERTIERT" : "NORMAL");
  diagnosticServer.sendHeader("Location", "/#nasa-raw");
  diagnosticServer.sendHeader("Cache-Control", "no-store");
  diagnosticServer.send(303, "text/plain", "NASA-Modus geaendert");
}

void appendForwarderCard(String &html, const NmeaUdpForwarder &forwarder) {
  html += F("<section><h2>WLAN-Ausgang ");
  html += forwarder.sourceName();
  html += F("</h2><dl><dt>Ziel</dt><dd>");
  html += SIGNALK_HOST.toString();
  html += ':';
  html += forwarder.destinationPort();
  html += F("</dd><dt>UDP-Status</dt><dd>");
  html += forwarder.isReady() ? F("bereit") : F("wartet auf WLAN");
  html += F("</dd><dt>Saetze gesendet</dt><dd>");
  html += forwarder.sentencesSent();
  html += F("</dd><dt>Saetze verworfen</dt><dd>");
  html += forwarder.sentencesDropped();
  html += F("</dd></dl></section>");
}

void handleDiagnosticPage() {
  String html;
  html.reserve(8500);
  html += F("<!doctype html><html lang='de'><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<meta http-equiv='refresh' content='3'><title>Lumi Instrument</title>"
            "<style>body{margin:0;background:#07110c;color:#e9f5ed;font-family:Verdana,sans-serif}"
            "main{max-width:920px;margin:auto;padding:24px}h1{color:#f4cf35}"
            ".meta{color:#a9c5b2;margin-bottom:20px}section{background:#10241a;border:1px solid #315c43;"
            "border-radius:12px;padding:18px;margin:14px 0}h2{margin-top:0;color:#67e28e}"
            "dl{display:grid;grid-template-columns:1fr 1fr;gap:8px 20px}dt{color:#a9c5b2}"
            "dd{margin:0;text-align:right;font-weight:bold}pre{white-space:pre-wrap;overflow-wrap:anywhere;"
            "background:#050a07;padding:12px;border-radius:8px;color:#fff}</style></head><body><main>"
            "<h1>Lumi Instrument Diagnostic</h1><div class='meta'>Firmware ");
  html += FIRMWARE_VERSION;
  html += F(" &middot; IP ");
  html += WiFi.localIP().toString();
  html += F(" &middot; WLAN ");
  html += WiFi.RSSI();
  html += F(" dBm &middot; Laufzeit ");
  html += millis() / 1000U;
  html += F(" s &middot; Aktualisierung alle 3 s</div>");
  appendReceiverCard(html, navmanReceiver);
  appendReceiverCard(html, nasaReceiver);
  html += F("<div id='nasa-raw'></div>");
  appendNasaDiagnostics(html);
  appendForwarderCard(html, navmanForwarder);
  appendForwarderCard(html, nasaForwarder);
  html += F("</main></body></html>");

  diagnosticServer.sendHeader("Cache-Control", "no-store");
  diagnosticServer.send(200, "text/html; charset=utf-8", html);
}

void startNetworkServices() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(WIFI_PASSWORD);
  ArduinoOTA.onStart([]() {
    Serial.println("[OTA] Update gestartet");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] Update abgeschlossen");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    const unsigned int percent = total == 0 ? 0 : (progress * 100U) / total;
    Serial.printf("\r[OTA] Fortschritt: %u%%", percent);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("\n[OTA] Fehler %u\n", static_cast<unsigned int>(error));
  });
  ArduinoOTA.begin();

  diagnosticServer.on("/", HTTP_GET, handleDiagnosticPage);
  diagnosticServer.on("/nasa-polarity", HTTP_POST, handleNasaPolarity);
  diagnosticServer.onNotFound([]() {
    diagnosticServer.send(404, "text/plain; charset=utf-8", "Nicht gefunden");
  });
  diagnosticServer.begin();
  networkServicesStarted = true;

  Serial.print("[WLAN] Diagnose: http://");
  Serial.print(WiFi.localIP());
  Serial.println('/');
  Serial.print("[OTA] Netzwerk-Port: ");
  Serial.print(OTA_HOSTNAME);
  Serial.println(" (Passwort entspricht dem WLAN-Passwort)");
}

void setup() {
  Serial.begin(USB_BAUD);

  // Serial0 is routed through the isolated onboard RS232 transceiver.
  Serial0.begin(NMEA_BAUD, SERIAL_8N1, NAVMAN_RX_PIN, NAVMAN_TX_PIN);
  // The external MAX490 receiver output is connected read-only to GPIO18.
  // Confirmed with the real NASA sender on 2026-10-18: inverted RX, CR endings.
  nasaSerial.begin(NMEA_BAUD, SERIAL_8N1, NASA_RX_PIN, -1, nasaInverted);

  delay(1000);
  Serial.println();
  Serial.print("Lumi Instrument Diagnostic v");
  Serial.println(FIRMWARE_VERSION);
  Serial.println("Nur Empfang: NAVMAN GPIO42, NASA GPIO18, jeweils 4800 Baud");
  Serial.print("Gueltige NAVMAN-Saetze gehen per UDP an ");
  Serial.print(SIGNALK_HOST);
  Serial.print(':');
  Serial.println(NAVMAN_UDP_PORT);
  Serial.print("Gueltige NASA-Saetze gehen per UDP an ");
  Serial.print(SIGNALK_HOST);
  Serial.print(':');
  Serial.println(NASA_UDP_PORT);

  WiFi.onEvent(handleWiFiEvent);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[WLAN] Verbinde mit: ");
  Serial.println(WIFI_SSID);
}

void loop() {
  navmanReceiver.poll();
  nasaReceiver.poll();

  if (WiFi.status() == WL_CONNECTED && !networkServicesStarted) {
    startNetworkServices();
  }
  if (networkServicesStarted) {
    ArduinoOTA.handle();
    diagnosticServer.handleClient();
  }

  const uint32_t now = millis();
  navmanForwarder.poll();
  nasaForwarder.poll();

  if (pendingDisconnectReason >= 0) {
    const int reason = pendingDisconnectReason;
    pendingDisconnectReason = -1;
    Serial.print("[WLAN] Abbruchgrund: ");
    Serial.println(reason);
  }

  static uint32_t lastStatusAt = 0;
  if (now - lastStatusAt >= STATUS_INTERVAL_MS) {
    lastStatusAt = now;
    printWiFiStatus();
    navmanReceiver.printStatus();
    nasaReceiver.printStatus();
    navmanForwarder.printStatus();
    nasaForwarder.printStatus();
  }

  delay(1);
}
