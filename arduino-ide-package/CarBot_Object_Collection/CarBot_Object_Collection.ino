/*
  ===============================================================================
  CarBot Object Collection System
  ===============================================================================
  
  An autonomous robot that scans for objects, grabs them with a servo gripper,
  and delivers them using L-shaped movement patterns.
  
  HARDWARE REQUIREMENTS:
  ----------------------
  - Arduino Uno
  - MPU6050 IMU (I2C: SDA=A4, SCL=A5)
  - TB6612FNG Motor Driver (or similar dual H-bridge)
  - HC-SR04 Ultrasonic Sensor
  - Servo Motor (for gripper)
  - Two DC Motors
  
  PIN CONNECTIONS:
  ----------------
  Motor Driver:
    - Left Motor PWM:  Pin 6
    - Right Motor PWM: Pin 5  
    - Left Direction:  Pin 8
    - Right Direction: Pin 7
    - Standby:         Pin 3
  
  Ultrasonic Sensor:
    - Trigger: Pin 13
    - Echo:    Pin 12
  
  Servo:
    - Control: Pin 10
  
  REQUIRED LIBRARIES:
  -------------------
  - Wire (built-in)
  - Servo (built-in)  
  - I2Cdevlib-MPU6050 (by Jeff Rowberg)
  - PID (by Brett Beauregard)
  
  FEATURES:
  ---------
  - Object scanning with 5° precision
  - Multi-reading sensor averaging for reliability
  - Two-stage PID turning system (coarse + fine control)
  - L-shaped delivery paths with alternating directions
  - Debug output control system
  - Memory-optimized for Arduino Uno
  
  AUTHOR: Robot Control System
  DATE: November 2025
  VERSION: 1.0
  ===============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include "move_PID.h"
#include "sonic.h" 
#include "servo_control.h"

// ===============================================================================
// MAIN ROBOT CLASS
// ===============================================================================

class MOVE_OBJECT {
private:
    // Hardware pin configuration
    const uint8_t vel_L = 6, vel_R = 5, dir_L = 8, dir_R = 7, stby = 3;
    const uint8_t USS_trig = 13, USS_listen = 12, max_sensor_distance = 40;
    const uint8_t servo_control_pin = 10;
    
    // Movement and scanning parameters (adjustable)
    const int object_grab_range = 5;      // Distance to grab object (cm)
    const int angle_increment = 10;       // Scan angle step (degrees)  
    const int max_scan_angle = 90;        // Max scan range (degrees)
    const int move_duration = 6000;       // Base movement time (ms)
    const int correction_time = 290;      // Path timing correction (ms)
    const int correction_angle = 8;       // Path angle correction (degrees)
    const double move_speed = 2.0;        // Movement speed (m/s)
    
    // State variables
    int theta = 0;                        // Current heading tracker
    bool path_direction;                  // true = left L-path, false = right L-path
    bool debug_enabled = false;           // Control debug output
    
    // Hardware objects
    RobotCarPID robot;
    SONICSENSE ultrasonic;
    SERVO_CONTROL servo_control;

public:
    // ===============================================================================
    // CONSTRUCTOR
    // ===============================================================================
    MOVE_OBJECT(bool direction = true) 
        : path_direction(direction)
        , robot(vel_L, vel_R, dir_L, dir_R, stby)
        , ultrasonic(USS_trig, USS_listen, max_sensor_distance)
        , servo_control(servo_control_pin) {}

    // ===============================================================================
    // INITIALIZATION
    // ===============================================================================
    bool initialize() {
        if (debug_enabled) Serial.println(F("Initializing..."));
        
        if (!robot.begin()) {
            Serial.println(F("Robot initialization failed"));
            return false;
        }
        
        robot.disableDebug();
        if (debug_enabled) Serial.println(F("Robot ready"));
        if (debug_enabled) Serial.println(F("System ready"));
        return true;
    }

    // ===============================================================================
    // SENSOR METHODS
    // ===============================================================================
    
    /**
     * Enhanced object detection with multiple readings for reliability
     * Takes 3 readings and averages valid ones to reduce noise
     * @return Average distance in cm, or -1 if no valid readings
     */
    int detect() {
        int readings[3];
        int valid_readings = 0;
        int sum = 0;
        
        for (int i = 0; i < 3; i++) {
            readings[i] = ultrasonic.check_range();
            delay(10); // Small delay between readings
            if (readings[i] > 0 && readings[i] <= max_sensor_distance) {
                sum += readings[i];
                valid_readings++;
            }
        }
        
        if (valid_readings > 0) {
            return sum / valid_readings; // Return average of valid readings
        }
        return -1; // No valid readings
    }

    // ===============================================================================
    // OBJECT DETECTION & SCANNING
    // ===============================================================================
    
    /**
     * Scans for objects using a two-phase approach:
     * Phase 1: Scan right side incrementally
     * Phase 2: Scan left side incrementally
     * @return true if object found and grabbed, false otherwise
     */
    bool scan_for_object() {
        if (debug_enabled) Serial.println(F("Scanning..."));
        
        // Phase 1: Scan right side
        for (int step = 0; step < max_scan_angle/2 / angle_increment; step++) {
            if (step > 0) {
                robot.turnToHeading(-angle_increment);
                theta -= angle_increment;
            }
            delay(100); // Sensor stabilization
            
            int dist = detect();
            if (debug_enabled) {
                Serial.print(F("R"));
                Serial.print(step);
                Serial.print(F(":"));
                Serial.println(dist);
            }
            
            if (dist > 0 && dist <= max_sensor_distance) {
                // Face object more precisely
                robot.turnToHeading(-angle_increment/2);
                theta -= angle_increment/2;
                delay(100);
                
                if (debug_enabled) {
                    Serial.print(F("OBJECT FOUND RIGHT: "));
                    Serial.print(dist);
                    Serial.println(F("cm - MOVING TO GRAB"));
                }
                return grab_object(-1);
            }
        }
        
        // Phase 2: Scan left side
        int total_left_turns = max_scan_angle / angle_increment;
        for (int step = 0; step < total_left_turns; step++) {
            robot.turnToHeading(angle_increment);
            theta += angle_increment;
            delay(200); // Longer stabilization for left scan
            
            int dist = detect();
            if (debug_enabled) {
                Serial.print(F("L"));
                Serial.print(step);
                Serial.print(F(":"));
                Serial.println(dist);
            }
            
            if (dist > 0 && dist <= max_sensor_distance) {
                // Face object more precisely
                robot.turnToHeading(angle_increment/2);
                theta += angle_increment;
                delay(100);
                
                if (debug_enabled) {
                    Serial.print(F("OBJECT FOUND LEFT: "));
                    Serial.print(dist);
                    Serial.println(F("cm - MOVING TO GRAB"));
                }
                return grab_object(1);
            }
        }
        return false; // No object found
    }

    // ===============================================================================
    // OBJECT MANIPULATION
    // ===============================================================================
    
    /**
     * Approaches and grabs detected object
     * @param sign Direction indicator (-1 for right, 1 for left)
     * @return true if object successfully grabbed
     */
    bool grab_object(int sign) {
        int current_distance = detect();
        if (current_distance == -1) return false;

        int distance_to_move = current_distance - object_grab_range;
        if (distance_to_move <= 0) {
            servo_control.close_servo();
            return true;
        }

        // Approach object incrementally
        unsigned long total_move_time = 0;
        while (detect() > object_grab_range && detect() != -1) {
            int remaining_distance = detect() - object_grab_range;
            int move_time = remaining_distance * 25; // ~25ms per cm
            
            robot.moveForTime(move_speed, move_time);
            total_move_time += move_time;
            delay(300); // Sensor settling
        }

        servo_control.close_servo(); // Grab object
        
        // Back away to scanning position
        if (total_move_time > 0) {
            robot.turnToHeading(sign * 185);  // Turn around
            robot.moveForTime(move_speed, total_move_time); // Move back
            robot.turnToHeading(sign * theta); // Reorient
        }
        
        return true;
    }

    // ===============================================================================
    // PATH EXECUTION
    // ===============================================================================
    
    /**
     * Executes L-shaped delivery path
     * Direction alternates automatically between calls
     */
    void execute_L_path() {
        // First leg of L-path
        robot.moveForTime(move_speed, move_duration);
        delay(300);
        
        // Turn (direction depends on path_direction)
        int turn_angle = (path_direction) ? 90 + correction_angle : -(90 - correction_angle);
        robot.turnToHeading(turn_angle);
        delay(300);
        
        // Second leg of L-path
        robot.moveForTime(move_speed, move_duration - correction_time);
        delay(300);
    }

    /**
     * Releases object and prepares for return journey
     */
    void release_object() {
        servo_control.open_servo();
        path_direction = !path_direction; // Alternate path direction
        robot.turnToHeading(180 + 10); // Turn to face pickup area
    }
    
    // ===============================================================================
    // UTILITY METHODS
    // ===============================================================================
    
    void switch_path_direction() {
        path_direction = !path_direction;
    }
    
    void enableDebug() {
        debug_enabled = true;
    }
    
    void disableDebug() {
        debug_enabled = false;
    }
};

// ===============================================================================
// GLOBAL OBJECTS & SETUP
// ===============================================================================

MOVE_OBJECT* robot_controller = nullptr;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Create robot object in setup() to avoid constructor hanging issues
    robot_controller = new MOVE_OBJECT(true);
    
    if (robot_controller == nullptr) {
        Serial.println(F("Object creation failed"));
        while(1) delay(1000);
    }
    
    // Enable debug for initial setup
    robot_controller->enableDebug();
    
    if (!robot_controller->initialize()) {
        Serial.println(F("Initialization failed"));
        while(1) delay(1000);
    }
    
    // Disable debug for normal operation (comment out to keep debug enabled)
    // robot_controller->disableDebug();
    
    Serial.println(F("System ready - Debug enabled for testing"));
}

// ===============================================================================
// MAIN LOOP
// ===============================================================================

void loop() {
    if (robot_controller->scan_for_object()) {
        // Object found and grabbed
        robot_controller->execute_L_path();  // Deliver object
        robot_controller->release_object();  // Release and turn around
        robot_controller->execute_L_path();  // Return to pickup area
        delay(1000); // Brief pause between cycles
    } else {
        // No object found, wait before next scan
        delay(2000);
    }
}

// ===============================================================================
// END OF MAIN SKETCH
// ===============================================================================