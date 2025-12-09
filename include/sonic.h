#include <Arduino.h>

// Ultrasonic sensor class for distance measurement

class SONICSENSE {
private:
    uint8_t trigPin;
    uint8_t echoPin;
    uint8_t range; // maximum range in cm

public:
    
    SONICSENSE(uint8_t trig, uint8_t echo, int range) : trigPin(trig), echoPin(echo), range(range) {
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

    int check_range() {
        int values_sum = 0;
        int values_count = 40; // <----- distance accuracy improved with more pings, but takes more time <-----
        for (int i = 0; i < values_count; i++) {
            values_sum += ping();
            delay(10);                              // Short delay between pings
        }
        int distance = values_sum / values_count;   // average of the values in the array
        if (distance > range) {                     // Cap the distance at the maximum range
            distance = -1;
        }
        return distance;
    }
    void setMaxRange(int max_range_cm) {
        range = max_range_cm;
    }
    int ping() {
        // Send a 10 microsecond pulse to trigger the ultrasonic sensor
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        long duration = pulseIn(echoPin, HIGH); // read the duration of the echo pulse
        int distance = duration * 0.034 / 2;    // convert to distance(cm)                       
        return distance;
    }


};
