#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Inicialización de sensores
Adafruit_MPU6050 mpu;
DHT dht(D2, DHT11); // Configura el pin y el tipo de sensor DHT11

// Configuración de BLE
#define SERVICE_UUID        "0311cee7-bb1b-48f3-a04f-594a1d848a69"
#define CHARACTERISTIC_UUID "b542fc7f-8106-4acf-8945-8eec57999381"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
String jsonData = "";

// Variables de almacenamiento de datos
unsigned long lastTime = 0;
const int dataInterval = 2000; // Intervalo de almacenamiento
int currentIndex = 0;
const int dataBufferSize = 5; // Tamaño del buffer para guardar muestras

struct SensorData {
    float gyroX, gyroY, gyroZ; // Inclinación en cada eje
    float temp; // Temperatura del DHT11
};

SensorData dataBuffer[dataBufferSize];

// Configuración del pulsador
const int buttonPin = 1;  // Pin digital para el pulsador
const int debounceDelay = 50; // Tiempo para evitar rebotes en ms

// Callbacks para BLE
class MyCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

void setup() {
    Serial.begin(115200);

    // Configurar el pulsador como entrada con resistencia pull-up
    pinMode(buttonPin, INPUT_PULLUP);

    // Inicialización de sensores
    if (!mpu.begin()) {
        Serial.println("Error al iniciar el MPU6050");
        while (1);
    }
    dht.begin();

    // Configuración de BLE
    BLEDevice::init("ESP32 BLE");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pCharacteristic->setValue("Hello World");
    pService->start();
    pServer->getAdvertising()->start();

    Serial.println("Dispositivo listo.");
}

void loop() {
    // Verificar si se presionó el pulsador
    if (digitalRead(buttonPin) == LOW) {
        delay(debounceDelay); // Manejo del rebote
        if (digitalRead(buttonPin) == LOW) {
            Serial.println("Entrando en modo Deep Sleep...");

            // Configurar el pin del pulsador como fuente de activación
            esp_sleep_enable_ext0_wakeup((gpio_num_t)buttonPin, 0); // Despierta con un LOW en el pin

            // Entrar en modo Deep Sleep
            esp_deep_sleep_start();
        }
    }

    // Lógica principal del programa
    if (millis() - lastTime > dataInterval) {
        lastTime = millis();

        // Leer datos del MPU6050
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Leer temperatura del DHT11
        float temperature = dht.readTemperature();

        // Guardar los datos en el buffer
        dataBuffer[currentIndex] = {g.gyro.x, g.gyro.y, g.gyro.z, temperature};
        currentIndex = (currentIndex + 1) % dataBufferSize;

        Serial.println("Datos almacenados:");
        Serial.printf("Gyro: X=%.2f, Y=%.2f, Z=%.2f, Temp=%.2f\n",
                      g.gyro.x, g.gyro.y, g.gyro.z, temperature);
    }
}
