#include <Arduino.h>
#include <Wire.h>
#include "move_PID.h"
#include "sonic.h"
#include "servo_control.h"

class MOVE_OBJECT {
private:
    const uint8_t vel_L = 6, vel_R = 5, dir_L = 8, dir_R = 7, stby = 3;
    const uint8_t USS_trig = 13, USS_listen = 12, max_sensor_distance = 40;
    const uint8_t servo_control_pin = 10;
    const int object_grab_range = 5, angle_increment = 10, max_scan_angle = 90;
    const int move_duration = 6000, correction_time = 290, correction_angle = 8;
    const double move_speed = 2.0;
    
    int theta = 0;
    bool path_direction;
    bool debug_enabled = false;  // Control debug output
    
    RobotCarPID robot;
    SONICSENSE ultrasonic;
    SERVO_CONTROL servo_control;

public:
    MOVE_OBJECT(bool direction = true) 
        : path_direction(direction)
        , robot(vel_L, vel_R, dir_L, dir_R, stby)
        , ultrasonic(USS_trig, USS_listen, max_sensor_distance)
        , servo_control(servo_control_pin) {}


    bool initialize() {
        if (debug_enabled) Serial.println(F("Initializing..."));
        
        if (!robot.begin()) {
            Serial.println(F("Robot initialization failed"));
            return false;
        }
        
        robot.disableDebug();
        if (debug_enabled) Serial.println(F("Robot ready"));
        
        // Servo is ready to use - no test needed
        if (debug_enabled) Serial.println(F("System ready"));
        return true;
    }

    int detect() {
        // Take multiple readings for better reliability with small objects
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

    bool scan_for_object() {
        if (debug_enabled) Serial.println(F("Scanning..."));
        
        // Scan right
        for (int step = 0; step < max_scan_angle/2 / angle_increment; step++) {
            if (step > 0) {
                robot.turnToHeading(-angle_increment);
                theta -= angle_increment;
            }
            delay(100); // Longer delay for sensor stabilization
            
            int dist = detect();
            if (debug_enabled) {
                Serial.print(F("R"));
                Serial.print(step);
                Serial.print(F(":"));
                Serial.println(dist);
            }
            
            if (dist > 0 && dist <= max_sensor_distance) {
                robot.turnToHeading(-angle_increment/2); // Face object
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
        
        // Scan left
        int total_left_turns = max_scan_angle / angle_increment;
        for (int step = 0; step < total_left_turns; step++) {
            robot.turnToHeading(angle_increment);
            theta += angle_increment;
            delay(200); // Longer delay for sensor stabilization
            
            int dist = detect();
            if (debug_enabled) {
                Serial.print(F("L"));
                Serial.print(step);
                Serial.print(F(":"));
                Serial.println(dist);
            }
            
            if (dist > 0 && dist <= max_sensor_distance) {
              robot.turnToHeading(angle_increment/2); // Face object
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
        return false;
    }

    bool grab_object(int sign) {
        int current_distance = detect();
        if (current_distance == -1) return false;

        int distance_to_move = current_distance - object_grab_range;
        if (distance_to_move <= 0) {
            servo_control.close_servo();
            return true;
        }

        unsigned long total_move_time = 0;
        while (detect() > object_grab_range && detect() != -1) {
            int remaining_distance = detect() - object_grab_range;
            int move_time = remaining_distance * 25;
            
            robot.moveForTime(move_speed, move_time);
            total_move_time += move_time;
            delay(300);
        }

        servo_control.close_servo();
        
        if (total_move_time > 0) {
            robot.turnToHeading(sign*185);
            robot.moveForTime(move_speed, total_move_time);
            robot.turnToHeading(sign*theta);
        }
        
        return true;
    }

    void execute_L_path() {
        robot.moveForTime(move_speed, move_duration);
        delay(300);
        
        int turn_angle = (path_direction) ? 90 + correction_angle : -(90 - correction_angle);
        robot.turnToHeading(turn_angle);
        delay(300);
        
        robot.moveForTime(move_speed, move_duration - correction_time);
        delay(300);
    }

    void release_object() {
        servo_control.open_servo();
        path_direction = !path_direction;
        robot.turnToHeading(180+10);
    }
    
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

MOVE_OBJECT* robot_controller = nullptr;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Create robot object
    robot_controller = new MOVE_OBJECT(true);
    
    if (robot_controller == nullptr) {
        Serial.println(F("Object creation failed"));
        while(1) delay(1000);
    }
    
    // Enable debug for initial setup, then disable for normal operation
    robot_controller->enableDebug();
    
    if (!robot_controller->initialize()) {
        Serial.println(F("Initialization failed"));
        while(1) delay(1000);
    }
    
    // Enable debug temporarily to test improved detection
    // robot_controller->disableDebug();  // Comment out to keep debug enabled
    
    Serial.println(F("System ready - Debug enabled for testing"));
}

void loop() {
    if (robot_controller->scan_for_object()) {
        robot_controller->execute_L_path();
        robot_controller->release_object();
        //robot_controller->switch_path_direction();
        robot_controller->execute_L_path();
        delay(1000);
    } else {
        delay(2000);
    }
}

