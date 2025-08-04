/*- Version: esp32_blynk_v1.0
- Simulates random sensor values (temperature, humidity, TDS, water level)
- Sends data every 5 seconds to Blynk (V0, V1, V2, V5)
- Removed UART communication and threshold update handling
- Removed all Blynk alerts and notification logic
- Used Blynk.virtualWrite() and Serial Monitor for output
- Useful for testing UI or debugging without hardware*/

// Blynk Config
#define BLYNK_TEMPLATE_ID "TMPL36mJcVg9w"
#define BLYNK_TEMPLATE_NAME "Hydroponic Farm Dashboard"
#define BLYNK_AUTH_TOKEN "H37uGturZZjeLsBgQwEm_YD-Aq3EOIjk"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Wi-Fi Credentials
char ssid[] = "realme";
char pass[] = "Sangita#2";

// Sensor data variables (simulated)
int temperature = 0;
int humidity = 0;
int tds = 0;
int waterLevel = 0;

BlynkTimer timer;

// Simulate random sensor values
void generateSensorData() {
  temperature = random(20, 40);     // °C
  humidity = random(40, 90);        // %
  tds = random(200, 600);           // ppm
  waterLevel = random(5, 100);      // %
}

// Send sensor data to Blynk
void sendSensorData()
{
  generateSensorData();

  Serial.println("---------------");
  Serial.printf("🌡 Temp: %d °C\n", temperature);
  Serial.printf("💧 Humidity: %d %%\n", humidity);
  Serial.printf("🧪 TDS: %d ppm\n", tds);
  Serial.printf("🌊 Water Level: %d %%\n", waterLevel);

  Blynk.virtualWrite(V0, temperature);
  Blynk.virtualWrite(V1, humidity);
  Blynk.virtualWrite(V2, tds);
  Blynk.virtualWrite(V5, waterLevel);

  Blynk.virtualWrite(V4, "✅ Simulated values sent.");
  Serial.println("📨 Sent to Blynk.");
}

// Setup
void setup()
{
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(5000L, sendSensorData);
  Serial.println("📡 ESP32 Simulation Ready.");
}

// Main loop
void loop()
{
  Blynk.run();
  timer.run();
}
