// Define trigger and echo pins for each sensor
const int trigPins[4] = {A3, 5, 2, A2};
const int echoPins[4] = {A4, 4, 3, A1};

// Sensor labels for serial output
const char* sensorNames[4] = {"fr_left", "fr_mid", "fr_right", "bc_mid"};

// Store distance values
float distances[4] = {0.0, 0.0, 0.0, 0.0};

// Maximum range in centimeters (4 meters)
const float MAX_RANGE_M = 4.0;

void setup() {
    Serial.begin(115200);
    
    // Initialize sensor pins
    for (int i = 0; i < 4; i++) {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
    }
}

void loop() {
    for (int i = 0; i < 4; i++) {
        distances[i] = measureDistance(trigPins[i], echoPins[i]);
        
        Serial.print(sensorNames[i]);
        Serial.print(":");
        Serial.println(distances[i]);
    }
    delay(500);
}

// Function to measure distance from ultrasonic sensor
float measureDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long timeElapsed = pulseIn(echoPin, HIGH, 23386);  // Timeout for 4m range

    // If timeout occurred, return max range
    if (timeElapsed == 0) {
        return MAX_RANGE_M;  // No echo received within max range
    }

    // Convert time to distance (cm)
    float distance = (float)timeElapsed / 1000000.0 / 2.0 * 343.0;

    // Cap the distance to 4 meters
    if (distance > MAX_RANGE_M) {
        distance = MAX_RANGE_M;
    }

    return distance;
}