#include <Arduino.h>
#include <Arduino_JSON.h>
#include <WebSocketsClient.h>
#include <WiFi.h>

// WiFi credentials (connect to the pattern controller's AP)
const char* ssid = "ReflectingThePresent";
const char* password = "lightshow2024";
const char* websocket_server = "192.168.4.1";
const int websocket_port = 81;

// WebSocket client
WebSocketsClient webSocket;

// Ultrasonic sensor configuration
#define NUM_SENSORS 2
#define READINGS_BUFFER_SIZE 5
#define TRIGGER_THRESHOLD_PERCENT 50

// Pin definitions for ultrasonic sensors (HC-SR04)
struct UltrasonicSensor {
    uint8_t trigger_pin;
    uint8_t echo_pin;
    uint8_t sensor_id;
    float readings[READINGS_BUFFER_SIZE];
    uint8_t reading_index;
    float average_distance;
    float max_distance;
    unsigned long last_reading_time;
    bool initialized;
};

// Define 8 ultrasonic sensors
UltrasonicSensor sensors[NUM_SENSORS] = {
    { 21, 22, 1, { 0 }, 0, 0, 0, 0, false }, // Sensor 1: Trigger=2, Echo=3
    { 23, 19, 2, { 0 }, 0, 0, 0, 0, false } // Sensor 2: Trigger=4, Echo=5
};

// // Define 8 ultrasonic sensors
// UltrasonicSensor sensors[NUM_SENSORS] = {
//     { 21, 22, 1, { 0 }, 0, 0, 0, false }, // Sensor 1: Trigger=2, Echo=3
//     { 23, 19, 2, { 0 }, 0, 0, 0, false }, // Sensor 2: Trigger=4, Echo=5
//     { 6, 7, 3, { 0 }, 0, 0, 0, false }, // Sensor 3: Trigger=6, Echo=7
//     { 8, 9, 4, { 0 }, 0, 0, 0, false }, // Sensor 4: Trigger=8, Echo=9
//     { 10, 11, 5, { 0 }, 0, 0, 0, false }, // Sensor 5: Trigger=10, Echo=11
//     { 12, 13, 6, { 0 }, 0, 0, 0, false }, // Sensor 6: Trigger=12, Echo=13
//     { 14, 15, 7, { 0 }, 0, 0, 0, false }, // Sensor 7: Trigger=14, Echo=15
//     { 16, 17, 8, { 0 }, 0, 0, 0, false } // Sensor 8: Trigger=16, Echo=17
// };

// Function declarations
void setupWiFi();
void setupWebSocket();
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);
void setupSensors();
float readUltrasonicDistance(UltrasonicSensor* sensor);
void updateSensorReadings();
void checkForTriggers();
void sendTriggerMessage(uint8_t sensor_id);

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== Reflecting The Present - Sensor Controller ===");

    // Setup ultrasonic sensors
    setupSensors();

    // Setup WiFi connection
    setupWiFi();

    // Setup WebSocket connection
    setupWebSocket();

    Serial.println("=== Sensor Controller Ready ===");
}

void loop()
{
    // Handle WebSocket events
    webSocket.loop();

    // Update sensor readings
    updateSensorReadings();

    // Check for distance triggers
    checkForTriggers();

    // Small delay to prevent overwhelming the system
    delay(50);
}

void setupWiFi()
{
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());
}

void setupWebSocket()
{
    webSocket.begin(websocket_server, websocket_port, "/");
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);
    Serial.println("WebSocket client initialized");
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length)
{
    switch (type) {
    case WStype_DISCONNECTED:
        Serial.println("WebSocket Disconnected");
        break;

    case WStype_CONNECTED:
        Serial.printf("WebSocket Connected to: %s\n", payload);
        break;

    case WStype_TEXT:
        Serial.printf("Received: %s\n", payload);
        break;

    default:
        break;
    }
}

void setupSensors()
{
    Serial.println("Initializing ultrasonic sensors...");

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensors[i].trigger_pin, OUTPUT);
        pinMode(sensors[i].echo_pin, INPUT);

        // Initialize readings buffer with default values
        for (uint8_t j = 0; j < READINGS_BUFFER_SIZE; j++) {
            sensors[i].readings[j] = 200.0; // Default to 200cm
        }

        sensors[i].reading_index = 0;
        sensors[i].average_distance = 200.0;
        sensors[i].max_distance = 0.0;
        sensors[i].last_reading_time = 0;
        sensors[i].initialized = false;

        Serial.print("Sensor ");
        Serial.print(sensors[i].sensor_id);
        Serial.print(" - Trigger: ");
        Serial.print(sensors[i].trigger_pin);
        Serial.print(", Echo: ");
        Serial.println(sensors[i].echo_pin);
    }

    Serial.println("Sensors initialized");
}

float readUltrasonicDistance(UltrasonicSensor* sensor)
{
    // Send trigger pulse
    digitalWrite(sensor->trigger_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(sensor->trigger_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensor->trigger_pin, LOW);

    // Read echo pulse
    unsigned long duration = pulseIn(sensor->echo_pin, HIGH, 30000); // 30ms timeout

    // Calculate distance in cm
    float distance = (duration * 0.034) / 2;

    // Validate reading (between 2cm and 400cm for HC-SR04)
    if (distance < 2.0 || distance > 400.0 || duration == 0) {
        distance = sensor->average_distance; // Use previous average if invalid
    }

    return distance;
}

void updateSensorReadings()
{
    unsigned long current_time = millis();

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        UltrasonicSensor* sensor = &sensors[i];

        // Read each sensor every 100ms
        if (current_time - sensor->last_reading_time >= 100) {
            sensor->last_reading_time = current_time;

            // Get new reading
            float new_distance = readUltrasonicDistance(sensor);

            // Add to circular buffer
            sensor->readings[sensor->reading_index] = new_distance;
            sensor->reading_index = (sensor->reading_index + 1) % READINGS_BUFFER_SIZE;

            // Calculate new average
            float sum = 0;
            for (uint8_t j = 0; j < READINGS_BUFFER_SIZE; j++) {
                sum += sensor->readings[j];
            }
            sensor->average_distance = sum / READINGS_BUFFER_SIZE;

            // Mark as initialized after first full buffer
            if (!sensor->initialized) {
                static uint8_t readings_count[NUM_SENSORS] = {0};
                readings_count[i]++;
                if (readings_count[i] >= READINGS_BUFFER_SIZE) {
                    sensor->initialized = true;
                    sensor->max_distance = sensor->average_distance;
                    Serial.print("Sensor ");
                    Serial.print(sensor->sensor_id);
                    Serial.print(" initialized with max distance: ");
                    Serial.print(sensor->max_distance);
                    Serial.println(" cm");
                }
            }
        }
    }
}

void checkForTriggers()
{
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        UltrasonicSensor* sensor = &sensors[i];

        // Only check initialized sensors
        if (!sensor->initialized)
            continue;

        // Get current reading (most recent)
        uint8_t current_index = (sensor->reading_index + READINGS_BUFFER_SIZE - 1) % READINGS_BUFFER_SIZE;
        float current_distance = sensor->readings[current_index];

        // Calculate threshold (50% of max distance)
        float trigger_threshold = sensor->max_distance * 0.5;

        // Only trigger when distance drops below 50% of max AND is decreasing
        if (current_distance < trigger_threshold && current_distance < sensor->average_distance) {
            Serial.print("TRIGGER! Sensor ");
            Serial.print(sensor->sensor_id);
            Serial.print(" - Distance: ");
            Serial.print(current_distance);
            Serial.print(" cm, Max: ");
            Serial.print(sensor->max_distance);
            Serial.print(" cm, Threshold: ");
            Serial.print(trigger_threshold);
            Serial.println(" cm");

            sendTriggerMessage(sensor->sensor_id);
        }
    }
}

void sendTriggerMessage(uint8_t sensor_id)
{
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected, cannot send trigger");
        return;
    }

    // Create JSON message
    JSONVar message;
    message["sensorId"] = sensor_id;
    message["timestamp"] = millis();

    String jsonString = JSON.stringify(message);

    // Send via WebSocket
    webSocket.sendTXT(jsonString);

    Serial.print("Sent trigger message: ");
    Serial.println(jsonString);
}