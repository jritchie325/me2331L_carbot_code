/*
  ===============================================================================
  sonic.h - Ultrasonic Distance Sensor Library
  ===============================================================================
  
  Simple and reliable ultrasonic distance measurement using HC-SR04 or compatible
  sensors. Features multiple reading averaging for improved accuracy.
  
  HARDWARE:
  - HC-SR04 Ultrasonic Sensor
  - Trigger pin: Digital output
  - Echo pin: Digital input
  
  FEATURES:
  - Multiple ping averaging for noise reduction
  - Configurable maximum range
  - Timeout protection
  - Simple interface
  
  USAGE:
    SONICSENSE sensor(13, 12, 30);  // Trigger=13, Echo=12, MaxRange=30cm
    int distance = sensor.check_range();
  
  ===============================================================================
*/

#ifndef SONIC_H
#define SONIC_H

#include <Arduino.h>

class SONICSENSE {
private:
    uint8_t trigPin;    // Trigger pin for ultrasonic sensor
    uint8_t echoPin;    // Echo pin for ultrasonic sensor  
    uint8_t range;      // Maximum detection range in cm

public:
    /**
     * Constructor for ultrasonic sensor
     * @param trig Trigger pin number
     * @param echo Echo pin number
     * @param max_range Maximum detection range in cm
     */
    SONICSENSE(uint8_t trig, uint8_t echo, int max_range) 
        : trigPin(trig), echoPin(echo), range(max_range) {
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

    /**
     * Get distance measurement with averaging for reliability
     * Takes multiple readings and averages them to reduce noise
     * @return Distance in cm, or -1 if out of range/no detection
     */
    int check_range() {
        int values_sum = 0;
        int values_count = 40;  // Number of pings to average (tunable)
        
        for (int i = 0; i < values_count; i++) {
            values_sum += ping();
            delay(10);  // Small delay between pings
        }
        
        int distance = values_sum / values_count;  // Average distance
        
        if (distance > range) {
            distance = -1;  // Out of range indicator
        }
        
        return distance;
    }

    /**
     * Set maximum detection range
     * @param max_range_cm Maximum range in centimeters
     */
    void setMaxRange(int max_range_cm) {
        range = max_range_cm;
    }

    /**
     * Single ping measurement
     * @return Distance in cm for single measurement
     */
    int ping() {
        // Send 10 microsecond trigger pulse
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        
        // Measure echo pulse duration
        long duration = pulseIn(echoPin, HIGH);
        
        // Convert to distance in cm
        // Speed of sound = 343 m/s = 0.0343 cm/μs
        // Distance = (duration * speed) / 2 (round trip)
        int distance = duration * 0.034 / 2;
        
        return distance;
    }

    /**
     * Quick range check without averaging (faster but less reliable)
     * @return Distance in cm, or -1 if out of range
     */
    int quick_check() {
        int distance = ping();
        return (distance > range) ? -1 : distance;
    }

    /**
     * Check if object is within specified distance
     * @param target_distance Distance threshold in cm
     * @return true if object detected within range
     */
    bool object_detected(int target_distance) {
        int distance = check_range();
        return (distance > 0 && distance <= target_distance);
    }

    /**
     * Get current maximum range setting
     * @return Maximum range in cm
     */
    int getMaxRange() {
        return range;
    }
};

#endif // SONIC_H