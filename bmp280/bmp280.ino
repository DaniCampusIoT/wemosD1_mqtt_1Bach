/*
 * ESP8266 + BMP280 -> MQTT (JSON cada 5s)
 *
 * Flujo general:
 *   setup():
 *     1) Inicializa Serial e I2C (Wire)
 *     2) Conecta a WiFi
 *     3) Configura MQTT + reconecta con LWT (Online/Offline)
 *     4) Escanea I2C (diagnóstico) e inicializa BMP280
 *   loop():
 *     1) Mantiene WiFi/MQTT conectados (reintentos)
 *     2) client.loop() para mantener viva la sesión MQTT
 *     3) Cada 5s lee BMP280 y publica JSON a un topic MQTT
 */

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define SEALEVELPRESSURE_HPA (1013.25)  // Presión a nivel del mar (hPa) usada para estimar altitud

// =====================
// Configuración general
// =====================
#define SET_SECURE_WIFI false  // false: MQTT sin TLS (LAN). true: MQTT con TLS (WiFiClientSecure)

#define WIFI_SSID       "HortSost"
#define WIFI_PASSWORD   "9b11c2671e5b"

// “Tipo” de nodo para componer topics (p.ej: cabrerapinto/meteorologia/...)
#define TYPE_NODE       String("meteorologia")

// =====================
// Configuración MQTT
// =====================
#if SET_SECURE_WIFI == false
  #include <WiFiClient.h>
  WiFiClient espClient;

  #define MQTT_SERVER   "192.168.1.101"
  #define MQTT_PORT     1883
  #define MQTT_USER     NULL
  #define MQTT_PASSWORD NULL
#else
  #include <WiFiClientSecure.h>
  BearSSL::WiFiClientSecure espClient;

  #define MQTT_SERVER   "huertociencias.uma.es"
  #define MQTT_PORT     8163
  #define MQTT_USER     "huerta"
  #define MQTT_PASSWORD "accesohuertica"
#endif

// Cliente MQTT montado sobre el cliente TCP (WiFiClient o WiFiClientSecure)
PubSubClient client(espClient);

// Instancia del sensor BMP280 (I2C)
Adafruit_BMP280 bmp;

// =====================
// Temporización
// =====================
unsigned long lastSendMs = 0;                 // (estado) último envío realizado
const unsigned long SEND_PERIOD_MS = 5000;    // período de envío (ms)

// ------------------------------------------------------------
// printSensorJson()
// ------------------------------------------------------------
/*
 * Uso:
 *   Función de debug: lee el BMP280, construye un JSON y lo imprime por Serial en formato "pretty".
 *
 * Entradas:
 *   - No recibe parámetros.
 *   - Usa como entradas implícitas (globales): `bmp` y `SEALEVELPRESSURE_HPA`.
 *
 * Salidas:
 *   - No devuelve nada (void).
 *   - Escribe por Serial un JSON legible.
 *
 * Comentarios interesantes:
 *   - Si `bmp.begin()` no se ejecutó o falló, las lecturas pueden ser 0/NaN o valores incoherentes.
 *   - `millis()` es un timestamp relativo desde el arranque (no es hora real).
 */
void printSensorJson() {
  // Lecturas del sensor
  float tempC   = bmp.readTemperature();                 // ºC
  float pressHP = bmp.readPressure() / 100.0;            // Pa -> hPa
  float altM    = bmp.readAltitude(SEALEVELPRESSURE_HPA);// m (estimada, depende de SEALEVELPRESSURE_HPA)

  // Documento JSON en RAM dinámica (heap)
  DynamicJsonDocument doc(1024);

  // Campos raíz
  doc["sensor"] = "BMP280";
  doc["timestamp"] = millis();

  // Objeto anidado "values"
  JsonObject values = doc.createNestedObject("values");
  values["temperatura_c"] = tempC;
  values["presion_hpa"]   = pressHP;
  values["altitud_m"]     = altM;

  // Imprime JSON formateado (con saltos/indentación)
  serializeJsonPretty(doc, Serial);
  Serial.println();
}

// ------------------------------------------------------------
// i2cScan()
// ------------------------------------------------------------
/*
 * Uso:
 *   Escáner I2C: recorre direcciones 0x01..0x7E e imprime qué dispositivos responden.
 *
 * Entradas:
 *   - No recibe parámetros.
 *   - Usa como entrada implícita: bus I2C inicializado con `Wire.begin()`.
 *
 * Salidas:
 *   - No devuelve nada (void).
 *   - Escribe por Serial direcciones encontradas y conteo total.
 *
 * Comentarios interesantes:
 *   - Muy útil para depurar cableado, pull-ups, o direcciones I2C (BMP280 suele ser 0x76/0x77).
 *   - Un resultado "Devices=0" suele indicar SDA/SCL mal, sin alimentación o sin GND común.
 */
void i2cScan() {
  Serial.println("[I2C] scan...");
  uint8_t count = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {        // 0 => ACK (dispositivo presente)
      Serial.printf("[I2C] Found 0x%02X\n", addr);
      count++;
      delay(2);                               // pequeña pausa para estabilidad en el bus
    }
  }
  Serial.printf("[I2C] Devices=%u\n", count);
}

// ------------------------------------------------------------
// iniciaSensor()
// ------------------------------------------------------------
/*
 * Uso:
 *   Inicializa el BMP280 probando primero 0x76 y luego 0x77. Si inicializa, configura oversampling/filtro.
 *
 * Entradas:
 *   - No recibe parámetros.
 *   - Usa como entradas implícitas: bus I2C (Wire) y objeto global `bmp`.
 *
 * Salidas:
 *   - No devuelve nada (void).
 *   - Deja el sensor listo para lecturas (si `bmp.begin()` devuelve true).
 *   - Escribe logs por Serial con el resultado.
 *
 * Comentarios interesantes:
 *   - La configuración de sampling se ajusta para lecturas "suaves" y estables.
 *   - Si no se detecta, el resto del programa seguirá (pero publicará valores inválidos).
 */
void iniciaSensor() {
  Serial.println("\n[BMP280] Init 0x76...");
  if (!bmp.begin(0x76)) {
    Serial.println("[BMP280] Fail 0x76, try 0x77...");
    if (!bmp.begin(0x77)) {
      Serial.println("*** BMP280 NO DETECTADO ***");
      return;
    }
  }

  Serial.println("[BMP280] Inicializado!");

  // Ajustes típicos: modo normal, oversampling y filtro para mejorar estabilidad
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,      // Modo continuo
    Adafruit_BMP280::SAMPLING_X2,      // Oversampling temperatura
    Adafruit_BMP280::SAMPLING_X16,     // Oversampling presión
    Adafruit_BMP280::FILTER_X16,       // Filtro IIR
    Adafruit_BMP280::STANDBY_MS_500    // Standby entre medidas internas
  );
}

// ------------------------------------------------------------
// wifiConnect()
// ------------------------------------------------------------
/*
 * Uso:
 *   Conecta el ESP8266 al WiFi en modo estación (STA). Reintenta si tarda más de 20s.
 *
 * Entradas:
 *   - No recibe parámetros.
 *   - Usa como entradas implícitas: WIFI_SSID, WIFI_PASSWORD.
 *
 * Salidas:
 *   - No devuelve nada (void).
 *   - Al terminar, WiFi queda conectado (o la función sigue reintentando).
 *   - Escribe por Serial el progreso, IP asignada y RSSI.
 *
 * Comentarios interesantes:
 *   - Es una función bloqueante: hasta que conecte no sale (con reintentos).
 *   - El RSSI (dBm) ayuda a diagnosticar cobertura (-30 excelente, -70 justo, -85 muy malo).
 */
void wifiConnect() {
  WiFi.mode(WIFI_STA);

  Serial.printf("\n[WiFi] Connecting to %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");

    // Si tarda demasiado, reinicia el intento
    if (millis() - t0 > 20000) {
      Serial.println("\n[WiFi] timeout, retry");
      WiFi.disconnect();
      delay(300);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      t0 = millis();
    }
  }

  Serial.printf("\n[WiFi] OK IP=%s RSSI=%d\n",
                WiFi.localIP().toString().c_str(),
                WiFi.RSSI());
}

// ------------------------------------------------------------
// reconnectMQTT()
// ------------------------------------------------------------
/*
 * Uso:
 *   Reconecta al broker MQTT en bucle hasta conseguir conexión.
 *   Configura LWT (Last Will): si el dispositivo cae sin desconexión limpia,
 *   el broker publicará "Offline" en el topic de conexión.
 *
 * Entradas:
 *   - No recibe parámetros.
 *   - Usa como entradas implícitas:
 *       MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASSWORD,
 *       TYPE_NODE, ESP.getChipId(), objeto global `client`.
 *
 * Salidas:
 *   - No devuelve nada (void).
 *   - Deja `client.connected()==true` cuando termina.
 *   - Publica "Online" retained en el topic de conexión al conectar.
 *
 * Comentarios interesantes:
 *   - clientId incluye el ChipId para evitar colisiones (dos nodos con mismo ID desconectan al otro).
 *   - El topic de conexión permite “monitoring” (Online/Offline) en dashboards.
 */
void reconnectMQTT() {
  // clientId único por chip
  String clientId = "ESP8266Client-";
  clientId += String(ESP.getChipId());

  // Topic de estado (Online/Offline)
  String topic_connect = ("cabrerapinto/" + String(TYPE_NODE) + "/" + clientId + "/connection");
  const char* conexion_topic = topic_connect.c_str();

  Serial.println("[MQTT] reconnect loop...");
  while (!client.connected()) {
    Serial.printf("[MQTT] Connecting to %s:%d as %s\n",
                  MQTT_SERVER, MQTT_PORT, clientId.c_str());

    /*
     * connect(clientID, user, pass, willTopic, willQoS, willRetain, willMessage, cleanSession)
     * - willMessage="Offline" retained => si el broker detecta caída, deja el estado Offline
     * - luego, al conectar bien, publicamos "Online" también retained
     */
    bool ok = client.connect(
      clientId.c_str(),
      MQTT_USER,
      MQTT_PASSWORD,
      conexion_topic,  // willTopic
      2,               // willQoS (0..2)
      true,            // willRetain
      "Offline",       // willMessage
      true             // cleanSession
    );

    if (ok) {
      Serial.println("[MQTT] connected");
      client.publish(conexion_topic, "Online", true);  // retained
    } else {
      Serial.printf("[MQTT] failed rc=%d, retry 5s\n", client.state());
      delay(5000);
    }
  }
}

// ------------------------------------------------------------
// sendToBroker()
// ------------------------------------------------------------
/*
 * Uso:
 *   Lee el BMP280, construye un JSON compacto, y lo publica en un topic MQTT.
 *
 * Entradas:
 *   - No recibe parámetros.
 *   - Usa como entradas implícitas: `bmp`, `client`, WiFi.localIP(), WiFi.RSSI(), TYPE_NODE, ESP.getChipId().
 *
 * Salidas:
 *   - No devuelve nada (void).
 *   - Publica MQTT (payload JSON) y escribe logs por Serial.
 *
 * Comentarios interesantes:
 *   - El topic usa el ChipId en HEX para identificar el nodo.
 *   - `serializeJson()` genera JSON minificado (sin espacios), ideal para MQTT.
 *   - `client.setBufferSize(1024)` en setup asegura que PubSubClient pueda manejar este payload.
 */
void sendToBroker() {
  // Topic de publicación final
  String pub_topic = ("cabrerapinto/" + String(TYPE_NODE) + "/" + String(ESP.getChipId(), HEX) + "/bmp280");
  const char* send_topic = pub_topic.c_str();

  // Lecturas
  float tempC   = bmp.readTemperature();
  float pressHP = bmp.readPressure() / 100.0;              // hPa
  float altM    = bmp.readAltitude(SEALEVELPRESSURE_HPA);  // m

  // Construcción JSON
  DynamicJsonDocument doc(1024);

  // Sub-objeto con info del ESP
  JsonObject esp = doc.createNestedObject("esp");
  esp["ip"]   = WiFi.localIP().toString();
  esp["rssi"] = WiFi.RSSI();  
  esp["timestamp"] = millis();

  // Valores del sensor
  JsonObject values = doc.createNestedObject("sensor");
  values["name"] = "BMP280";
  values["temperatura_c"] = tempC;
  values["presion_hpa"]   = pressHP;
  values["altitud_m"]     = altM;

  // Serializa JSON a un buffer C (char[]) para publicarlo
  char payload[1024];
  size_t n = serializeJson(doc, payload, sizeof(payload));

  // Asegura terminación nula para imprimir como string sin basura
  payload[(n < sizeof(payload)) ? n : (sizeof(payload) - 1)] = '\0';

  // Log local
  Serial.printf("\n[PUB] %s len=%u\n", send_topic, (unsigned)n);
  Serial.println(payload);

  // Garantiza conexión MQTT antes de publicar
  if (!client.connected()) reconnectMQTT();

  // Publicación (por defecto sin retain; puedes cambiarlo si quieres)
  bool ok = client.publish(send_topic, payload);
  Serial.printf("[PUB] publish=%s\n", ok ? "OK" : "FAIL");
}

// ------------------------------------------------------------
// setup()
// ------------------------------------------------------------
/*
 * Uso:
 *   Inicialización del sistema (se ejecuta 1 vez al arrancar).
 *
 * Entradas:
 *   - No recibe parámetros.
 *
 * Salidas:
 *   - No devuelve nada (void).
 *   - Deja WiFi conectado, MQTT conectado, sensor inicializado (si está bien cableado).
 *
 * Comentarios interesantes:
 *   - `Wire.begin()` inicia I2C con los pines por defecto de la placa/board seleccionado.
 *   - `client.setBufferSize(1024)` debe ser >= tamaño máximo del JSON para no truncar.
 */
void setup() {
  Serial.begin(115200);

  // Inicia bus I2C (en ESP8266 puedes usar Wire.begin(SDA, SCL) si quieres pines custom)
  Wire.begin();

  Serial.println("\n=== NodeMCU BMP280 -> MQTT (cada 5s) ===");

  // 1) WiFi
  wifiConnect();

  // 2) MQTT
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setBufferSize(1024);
  reconnectMQTT();

  // 3) Diagnóstico + sensor
  i2cScan();
  iniciaSensor();

  // 4) Inicializa temporizador de publicación
  lastSendMs = millis();

  Serial.println("Setup done");
}

// ------------------------------------------------------------
// loop()
// ------------------------------------------------------------
/*
 * Uso:
 *   Bucle principal (se ejecuta continuamente).
 *
 * Entradas:
 *   - No recibe parámetros.
 *
 * Salidas:
 *   - No devuelve nada (void).
 *   - Mantiene WiFi/MQTT operativos y publica datos cada SEND_PERIOD_MS.
 *
 * Comentarios interesantes:
 *   - `client.loop()` es importante para mantener viva la conexión MQTT y procesar ACKs.
 *   - El envío periódico se basa en `millis()` para evitar delays largos.
 */
void loop() {
  // 1) Asegura WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] lost -> reconnect");
    wifiConnect();
  }

  // 2) Asegura MQTT
  if (!client.connected()) {
    Serial.println("[MQTT] lost -> reconnect");
    reconnectMQTT();
  }

  // 3) Mantiene la sesión MQTT
  client.loop();

  // 4) Publicación periódica
  unsigned long now = millis();
  if (now - lastSendMs >= SEND_PERIOD_MS) {
    lastSendMs = now;
    sendToBroker();
  }
}
