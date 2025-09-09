/*
  gowoon, ESP32 — UDP ADC
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"
#include <driver/adc.h>

/* ---------- user parameters ---------- */
#define RATE_HZ        125                // packets / samples per second
#define DEST_IP        "192.168.2.2"      // PC IPv4 (edit!)
#define DEST_PORT      7006
#define LOCAL_PORT     7000
#define LOG_EVERY_MS   1000               // serial stats period (ms)

/* ---------- ADC parameters ---------- */
#define ADC_PIN        33                 // GPIO33
#define ADC_PIN2       32                 // GPIO35
#define ADC_WIDTH_BIT  12                 // up to 12 bits on ESP32
#define ADC_ATTEN      ADC_11db           // 0-~3.6 V full-scale

const char *SSID = "meric_drone";
const char *PWD  = "meric160816";

WiFiUDP   udp;
IPAddress destIp;

void setup() {
  Serial.begin(115200);

  /* Wi-Fi STA */
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(SSID, PWD);
  while (WiFi.status() != WL_CONNECTED) delay(10);
  Serial.printf("[Wi-Fi] ESP IP: %s\n", WiFi.localIP().toString().c_str());

  /* Wi-Fi performance tweaks */
  esp_wifi_set_ps(WIFI_PS_NONE);                // disable power save
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20); // classic ESP32 only HT20

  /* UDP socket */
  udp.begin(LOCAL_PORT);
  Serial.println("[UDP] socket open");

  /* Convert DEST_IP string → IPAddress */
  destIp.fromString(DEST_IP);

  /* ADC setup */
  analogReadResolution(ADC_WIDTH_BIT);          
  analogSetPinAttenuation(ADC_PIN, ADC_ATTEN);  
  analogSetPinAttenuation(ADC_PIN2, ADC_ATTEN); 
}

void loop() {
  static uint32_t seq     = 0;
  static uint32_t lastLog = millis();

  /* ---- read ADC & send ---- */
  int adcRaw  = analogRead(ADC_PIN);        
  int adcRaw2 = analogRead(ADC_PIN2);       

  char buf[32];
  int n = snprintf(buf, sizeof(buf),
                   "SEQ:%lu adc:%d", seq, -adcRaw + adcRaw2);

  udp.beginPacket(destIp, DEST_PORT);
  udp.write(reinterpret_cast<uint8_t*>(buf), n);
  bool ok = udp.endPacket();
  seq++;

  /* ---- periodic serial log ---- */
  uint32_t now = millis();
  if (now - lastLog >= LOG_EVERY_MS) {
    Serial.printf("TX rate: %lu fps, last packet %s, adc=%d\n",
                  seq * 1000UL / (now + 1),
                  ok ? "OK" : "FAIL", -adcRaw + adcRaw2);
    lastLog = now;
    seq     = 0;
  }

  /* ---- precise pacing ---- */
  static uint32_t nextTick = 0;
  if (!nextTick) nextTick = micros();
  nextTick += 1000000UL / RATE_HZ;   // ← ' 제거
  int32_t dt = static_cast<int32_t>(nextTick - micros());
  if (dt > 0)   delayMicroseconds(dt);
  else          nextTick = micros();   // overrun → resync
}
