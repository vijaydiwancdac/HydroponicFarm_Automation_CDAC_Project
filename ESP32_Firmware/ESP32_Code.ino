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

// UART Configuration
#define STM_RX 16  // ESP32 receives from STM32 TX
#define STM_TX 17  // Not used for TX, but defined
HardwareSerial SerialSTM(2);  // UART2 on ESP32

// Sensor data variables (as integers)
int temperature = 0;
int humidity = 0;
int tds = 0;
int waterLevel = 0;

// Thresholds
const int TEMP_THRESHOLD = 34;
const int HUMIDITY_THRESHOLD = 100;
int tdsThreshold = 300;  // Default, can be updated via Blynk V3
int waterThreshold = 10; // Water level threshold

BlynkTimer timer;

// Receive updated TDS threshold from Blynk app (V3)
BLYNK_WRITE(V3)
{
  tdsThreshold = param.asInt();
  Serial.print("TDS Threshold updated from Blynk (V3): ");
  Serial.println(tdsThreshold);
 
  // Send float with two decimal places followed by newline
  SerialSTM.printf("%.2f\n", static_cast<float>(tdsThreshold));

}

// Send sensor data to Blynk and check thresholds
void sendSensorData()
{
  // Only proceed if valid data has been received
  if (temperature == 0 && humidity == 0 && tds == 0 && waterLevel == 0) {
    Serial.println("⚠ No sensor data yet from STM32. Skipping update.");
    return;
  }

  Serial.println("---------------");
  Serial.printf("🌡 Temp: %d °C\n", temperature);
  Serial.printf("💧 Humidity: %d %%\n", humidity);
  Serial.printf("🧪 TDS: %d ppm\n", tds);
  Serial.printf("🌊 Water Level: %d %%\n", waterLevel);
  Serial.printf("📊 TDS Threshold: %d\n", tdsThreshold);

  // Send data to Blynk virtual pins
  Blynk.virtualWrite(V0, temperature);
  Blynk.virtualWrite(V1, humidity);
  Blynk.virtualWrite(V2, tds);
  Blynk.virtualWrite(V5, waterLevel);

   String message = "";

  if (temperature > TEMP_THRESHOLD) {
    String msg = "🔥 High Temperature: " + String(temperature) + " °C";
    Blynk.logEvent("temperature_alert", msg);
    message += msg + "\n";
    Serial.println("🔥 Event: " + msg);
  }

  if (humidity > HUMIDITY_THRESHOLD) {
    String msg = "💧 High Humidity: " + String(humidity) + " %";
    Blynk.logEvent("humidity_alert", msg);
    message += msg + "\n";
    Serial.println("💧 Event: " + msg);
  }

  if (waterLevel < waterThreshold) {
    String msg = "🚱 Low Water Level: " + String(waterLevel) + " L";
    Blynk.logEvent("water_level_alert", msg);
    message += msg + "\n";
    Serial.println("🚱 Event: " + msg);
  }

  if (tds < tdsThreshold) { 
    int ppmDifference = tdsThreshold - tds;

    const int tankVolumeLiters = 1000; // Full tank size in liters
    int actualWaterLiters = (waterLevel * tankVolumeLiters) / 100;

    // Nutrient (in mL) = (ppmDifference * actualWaterLiters) / 500
    int requiredNutrientML = (ppmDifference * actualWaterLiters) / 500;

    String msg = "🌿 TDS low (" + String(tds) + " ppm). Add " + String(requiredNutrientML) + " mL nutrient to reach " + String(tdsThreshold) + " ppm.";
    Blynk.logEvent("tds_alert", msg);
    message += msg + "\n";
    Serial.println("🌿 Event: " + msg);
    }


  if (message == "") {
    message = "✅ All values normal.";
  }

  Blynk.virtualWrite(V4, message);
  Serial.println("📨 Summary Message:");
  Serial.println(message);

}

// Setup function
void setup()
{
  Serial.begin(115200);
  SerialSTM.begin(115200, SERIAL_8N1, STM_RX, STM_TX); // Start UART2

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  timer.setInterval(5000L, sendSensorData); // Send data every 5 seconds

  Serial.println("📡 ESP32 ready to receive from STM32 and update Blynk");
}

// Main loop
void loop()
{
  Blynk.run();
  timer.run();

  // Handle incoming UART data from STM32
  if (SerialSTM.available()) {
    String incomingLine = SerialSTM.readStringUntil('\n');
    incomingLine.trim();  // Remove whitespace

    if (incomingLine.length() > 0) {
      float tempF = 0.0, humF = 0.0, tdsF = 0.0, waterF = 0.0;

      // Parse STM32 data format: Temp,Humidity,TDS,WaterLevel
      int res = sscanf(incomingLine.c_str(), "%f,%f,%f,%f",
                       &tempF, &humF, &tdsF, &waterF);

      if (res == 4) {
        // Convert float to int
        temperature = static_cast<int>(tempF);
        humidity = static_cast<int>(humF);
        tds = static_cast<int>(tdsF);
        waterLevel = static_cast<int>(waterF);

        Serial.println("✅ Parsed data from STM32:");
        Serial.printf("🌡 %d°C | 💧 %d%% | 🧪 %d ppm | 🌊 %d%%\n", temperature, humidity, tds, waterLevel);
      } else {
        Serial.println("❌ Failed to parse STM32 data:");
        Serial.println(incomingLine);
      }
    }
  }
}