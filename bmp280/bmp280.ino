#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <ArduinoJson.h>   // <-- ArduinoJson

#define SEALEVELPRESSURE_HPA (1013.25)


Adafruit_BMP280 bmp;

// ===== Timing =====
unsigned long lastSendMs = 0;
const unsigned long SEND_PERIOD_MS = 5000;

// Encapsula los datos del sensor en JSON y los imprime 
void printSensorJson() {
  float tempC   = bmp.readTemperature();
  float pressHP = bmp.readPressure() / 100.0;              // hPa
  float altM    = bmp.readAltitude(SEALEVELPRESSURE_HPA);  // m

  // Me creo el documento donde voy a almacenar los datos
  DynamicJsonDocument doc(1024);  // 1024 bytes (suficiente para datos grandes)

  // Construir objeto JSON
  doc["sensor"] = "BMP280";
  doc["timestamp"] = millis();

  JsonObject values = doc.createNestedObject("values");
  values["temperatura_c"] = tempC;
  values["presion_hpa"]  = pressHP;
  values["altitud_m"]    = altM;

  // Salida pretty por Serial
  serializeJsonPretty(doc, Serial);
  Serial.println();
}

void iniciaSensor(){
  bool ok = bmp.begin(0x76);
  if (!ok) ok = bmp.begin(0x77);

  if (!ok) {
    Serial.println("No se detecta BMP280 (0x76/0x77). Revisa cableado/direccion.");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Mega: SDA=20, SCL=21
                // Arduino Uno: SDA=4, SCL=5

  iniciaSensor();
  lastSendMs = millis();
  Serial.println("Setup done");
}

void loop() {
  unsigned long now = millis();
  if (now - lastSendMs >= SEND_PERIOD_MS) {
    lastSendMs = now;
    printSensorJson();
  }
}
