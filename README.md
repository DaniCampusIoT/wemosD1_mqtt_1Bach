# ESP8266 + BMP280 → MQTT (JSON)

Proyecto didáctico de meteorología IoT para 1º de Bachillerato

## Descripción

Este repositorio contiene un proyecto en Arduino IDE para **ESP8266 (NodeMCU)** que lee un sensor barométrico **BMP280** por I2C (temperatura, presión y altitud estimada) y envía los datos en **formato JSON** a un **broker MQTT** cada 5 segundos.

Está pensado como práctica guiada para alumnado de 1º de Bachillerato: aprender I2C, sensores, redes WiFi, mensajería MQTT y estructuración de datos con JSON.

***

## Funcionalidad (qué hace)

- Se conecta a una red **WiFi** (con reintentos si falla).
- Se conecta a un **broker MQTT** (con reconexión automática).
- Publica un mensaje de estado **Online/Offline** usando LWT (Last Will) para monitorizar si el nodo cae.[^1]
- Inicializa y configura el **BMP280**.
- Cada `SEND_PERIOD_MS`:
    - Lee `temperatura_c`, `presion_hpa`, `altitud_m`.
    - Construye un JSON con:
        - Metadatos del nodo (IP, RSSI).
        - Timestamp relativo (`millis()`).
        - Valores del sensor.
    - Publica el JSON en un topic MQTT.

***

## Material necesario

- 1 × ESP8266 NodeMCU (o compatible)
- 1 × Sensor BMP280 por I2C
- Cables Dupont
- (Opcional) Resistencias pull-up I2C 4.7kΩ si el módulo no las incluye

***

## Conexiones (cableado)

### BMP280 (I2C) → NodeMCU (ESP8266)

- **VCC → 3V3**
- **GND → GND**
- **SDA → D2 (GPIO4)**
- **SCL → D1 (GPIO5)**

> Importante: usa 3.3V (no 5V) para evitar problemas o daños.

***

## Software y librerías

Instala en Arduino IDE (Library Manager):

- Adafruit BMP280 Library
- ArduinoJson
- PubSubClient
- (ESP8266 core instalado en el IDE)

***

## Configuración rápida (qué debes cambiar)

En la cabecera del código:

```cpp
#define WIFI_SSID     "TU_WIFI"
#define WIFI_PASSWORD "TU_PASS"

#define SET_SECURE_WIFI false

// Si SET_SECURE_WIFI == false:
#define MQTT_SERVER "192.168.1.81"
#define MQTT_PORT   1883
```

Otros parámetros útiles:

- `TYPE_NODE`: etiqueta para organizar topics (`meteorologia`, `aula1`, etc.).
- `SEND_PERIOD_MS`: periodo de publicación (por defecto 5000 ms).

***

## Topics MQTT y ejemplo de mensaje

### Topic de datos

El código publica en:

```
orchard/<TYPE_NODE>/<chipIdHex>/cjmcu8128
```

> Si tu proyecto es “solo BMP280”, puedes renombrar el sufijo `cjmcu8128` por `bmp280` en la función `sendToBroker()`.

### Ejemplo de payload JSON

```json
{
  "esp": { "ip": "10.0.0.15", "rssi": -67 },
  "sensor": "BMP280",
  "timestamp": 34795,
  "values": {
    "temperatura_c": 22.31,
    "presion_hpa": 1012.84,
    "altitud_m": 12.7
  }
}
```


### Topic de estado (conexión)

Se publica un estado retained en:

```
orchard/<TYPE_NODE>/ESP8266Client-<chipId>/connection
```

- Al conectar: `"Online"` (retained)
- Si cae sin desconectar bien: `"Offline"` (LWT, retained)[^1]

***

## Estructura del código (para estudiantes)

Funciones principales:

- `wifiConnect()`: conecta a WiFi (bloqueante con reintentos).
- `reconnectMQTT()`: conecta a MQTT y configura LWT + publica “Online”.
- `i2cScan()`: escáner I2C para verificar que el sensor responde (diagnóstico).
- `iniciaSensor()`: inicializa el BMP280 (0x76/0x77) y configura el muestreo.
- `sendToBroker()`: lee sensor → crea JSON → publica por MQTT.
- `loop()`: mantiene conexiones y ejecuta el envío periódico.

***

## Cómo modificar sensores y enviar otros datos

La parte “educativa” del proyecto es que el alumnado pueda **cambiar el sensor** o **añadir más variables**. Estas son las zonas que deben tocar:

### 1) Añadir una nueva librería / objeto del sensor

- En la parte de `#include` (arriba).
- Crear una instancia global (igual que `Adafruit_BMP280 bmp;`).


### 2) Inicialización del sensor

- En `setup()` o en una nueva función tipo `iniciaSensorX()`.
- Patrón recomendado:
    - `i2cScan();` (si es I2C)
    - `sensor.begin(...)` y logs por Serial.


### 3) Lectura y publicación (la parte clave)

En `sendToBroker()`:

- Sustituir estas lecturas:

```cpp
float tempC   = bmp.readTemperature();
float pressHP = bmp.readPressure() / 100.0;
float altM    = bmp.readAltitude(SEALEVELPRESSURE_HPA);
```

- Y modificar el JSON en:

```cpp
JsonObject values = doc.createNestedObject("values");
values["temperatura_c"] = tempC;
values["presion_hpa"]   = pressHP;
values["altitud_m"]     = altM;
```


#### Ejemplo: añadir “luz” (LDR analógico) o “humedad”

- Lees el nuevo dato (p.ej. `int luz = analogRead(A0);`)
- Lo añades al JSON:

```cpp
values["luz_raw"] = luz;
```


### 4) Cambiar el topic de publicación (opcional)

También en `sendToBroker()`:

```cpp
String pub_topic = "orchard/" + TYPE_NODE + "/" + String(ESP.getChipId(), HEX) + "/bmp280";
```

Así cada sensor/proyecto queda bien organizado.

***

## Actividades sugeridas (aula)

- Cambiar `SEND_PERIOD_MS` y observar carga de red/broker.
- Añadir una variable nueva en `values` (y verla en el suscriptor MQTT).
- Calibrar altitud: ajustar `SEALEVELPRESSURE_HPA` y comprobar cambios.
- Diseñar un “dashboard” (Node-RED / Home Assistant / Grafana) con el topic.

***

## Resolución de problemas

- Si `i2cScan()` no encuentra `0x76` o `0x77`:
    - Revisa SDA/SCL, alimentación 3.3V y GND común.
- Si MQTT conecta pero no llegan datos:
    - Revisa `MQTT_SERVER`, puerto, y que el broker esté accesible desde la WiFi.
- Si el JSON es grande y falla `publish()`:
    - Asegura `client.setBufferSize(1024);` y reduce campos si hace falta.

***
