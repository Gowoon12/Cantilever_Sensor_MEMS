/*
  gowoon, ESP32 — UDP ADC (3 differential pairs)
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"
#include <driver/adc.h>

/* ---------- user parameters ---------- */
#define RATE_HZ        125                // packets / samples per second
#define DEST_IP        "192.168.2.2"      // PC IPv4 (edit!)
#define DEST_PORT      7003
#define LOCAL_PORT     7000
#define LOG_EVERY_MS   1000               // serial stats period (ms)

/* ---------- ADC parameters ---------- */
#define ADC_PINA1      33  // Pair A -
#define ADC_PINA2      32  // Pair A +
#define ADC_PINB1      35  // Pair B -
#define ADC_PINB2      34  // Pair B +
#define ADC_PINC1      39  // Pair C -
#define ADC_PINC2      36  // Pair C +
#define ADC_WIDTH_BIT  12
#define ADC_ATTEN      ADC_11db           // 0 ~ 3.6 V full-scale

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
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);

  /* UDP socket */
  udp.begin(LOCAL_PORT);
  Serial.println("[UDP] socket open");

  /* Convert DEST_IP string → IPAddress */
  destIp.fromString(DEST_IP);

  /* ADC setup */
  analogReadResolution(ADC_WIDTH_BIT);          
  analogSetPinAttenuation(ADC_PINA1, ADC_ATTEN);  
  analogSetPinAttenuation(ADC_PINA2, ADC_ATTEN); 
  analogSetPinAttenuation(ADC_PINB1, ADC_ATTEN);  
  analogSetPinAttenuation(ADC_PINB2, ADC_ATTEN); 
  analogSetPinAttenuation(ADC_PINC1, ADC_ATTEN);  
  analogSetPinAttenuation(ADC_PINC2, ADC_ATTEN); 
}

void loop() {
  static uint32_t seq     = 0;
  static uint32_t lastLog = millis();

  /* ---- read ADC pairs ---- */
  int a1 = analogRead(ADC_PINA1);
  int a2 = analogRead(ADC_PINA2);
  int b1 = analogRead(ADC_PINB1);
  int b2 = analogRead(ADC_PINB2);
  int c1 = analogRead(ADC_PINC1);
  int c2 = analogRead(ADC_PINC2);

  int diffA = a2 - a1;   // Pair A (32 - 33)
  int diffB = b2 - b1;   // Pair B (34 - 35)
  int diffC = c2 - c1;   // Pair C (36 - 39)

  /* ---- build packet ---- */
  char buf[64];
  int n = snprintf(buf, sizeof(buf),
                   "SEQ:%lu adc1:%d adc2:%d adc3:%d",
                   seq, diffA, diffB, diffC);

  udp.beginPacket(destIp, DEST_PORT);
  udp.write(reinterpret_cast<uint8_t*>(buf), n);
  bool ok = udp.endPacket();
  seq++;

  /* ---- periodic serial log ---- */
  uint32_t now = millis();
  if (now - lastLog >= LOG_EVERY_MS) {
    Serial.printf("TX rate: %lu fps, last packet %s, "
                  "adc1=%d adc2=%d adc3=%d\n",
                  seq * 1000UL / (now + 1),
                  ok ? "OK" : "FAIL",
                  diffA, diffB, diffC);
    lastLog = now;
    seq     = 0;
  }

  /* ---- precise pacing ---- */
  static uint32_t nextTick = 0;
  if (!nextTick) nextTick = micros();
  nextTick += 1000000UL / RATE_HZ;
  int32_t dt = static_cast<int32_t>(nextTick - micros());
  if (dt > 0)   delayMicroseconds(dt);
  else          nextTick = micros();   // overrun → resync
}
