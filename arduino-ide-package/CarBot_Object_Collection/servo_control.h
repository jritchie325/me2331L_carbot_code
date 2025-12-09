/*
  ===============================================================================
  servo_control.h - Servo Motor Control Library
  ===============================================================================
  
  Advanced servo control with smooth movement and customizable positions.
  Designed for gripper/manipulator applications with open/close functionality.
  
  HARDWARE:
  - Standard servo motor (SG90, MG996R, etc.)
  - Control signal: PWM pin
  - Power: 5V supply (separate from Arduino if high-torque servo)
  
  FEATURES:
  - Smooth gradual movement between positions
  - Customizable open/close positions
  - Adjustable movement speed
  - Proper pulse width limits for compatibility
  - Movement feedback via serial
  
  USAGE:
    SERVO_CONTROL gripper(10);  // Servo on pin 10
    gripper.close_servo();      // Close gripper
    gripper.open_servo();       // Open gripper
  
  ===============================================================================
*/

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Servo.h>
#include <Arduino.h>

class SERVO_CONTROL {
private:
    Servo servo_arm;                    // Servo object
    int servo_arm_open = 140;           // Open position angle (adjustable)
    int servo_arm_closed = 44;          // Closed position angle (adjustable)
    int servo_speed = 25;               // Movement speed (higher = slower)
    uint8_t servo_control_pin;          // Control pin number
    bool servo_attached = false;        // Attachment status

public:
    /**
     * Constructor - initializes servo on specified pin
     * @param servo_pin PWM pin for servo control
     */
    SERVO_CONTROL(uint8_t servo_pin) : servo_control_pin(servo_pin) {
        // Attach servo with proper pulse width limits for better compatibility
        servo_arm.attach(servo_control_pin, 500, 2400);
        servo_attached = true;
        delay(100);  // Brief delay for servo initialization
        
        Serial.print("Servo attached to pin: ");
        Serial.println(servo_control_pin);
    }

    /**
     * Set movement speed
     * @param speed_value Delay between steps (higher = slower)
     */
    void speed(int speed_value) {
        servo_speed = speed_value;
    }

    /**
     * Set custom open position
     * @param open_angle Angle for open position (0-180 degrees)
     */
    void set_open_value(int open_angle) {
        servo_arm_open = constrain(open_angle, 0, 180);
    }

    /**
     * Set custom close position  
     * @param close_angle Angle for closed position (0-180 degrees)
     */
    void set_close_value(int close_angle) {
        servo_arm_closed = constrain(close_angle, 0, 180);
    }

    /**
     * Move servo smoothly to target angle
     * @param target_angle Desired angle (0-180 degrees)
     */
    void servo_to_angle(int target_angle) {
        if (!servo_attached) {
            Serial.println("Error: Servo not attached!");
            return;
        }
        
        target_angle = constrain(target_angle, 0, 180);  // Safety limit
        
        Serial.print("Moving servo to angle: ");
        Serial.println(target_angle);
        
        int delay_time = 15;  // Base delay between steps
        int current_angle = servo_arm.read();
        int angle_diff = target_angle - current_angle;
        
        Serial.print("Current: ");
        Serial.print(current_angle);
        Serial.print("° -> Target: ");
        Serial.print(target_angle);
        Serial.println("°");

        // Smooth gradual movement
        while(abs(angle_diff) > 1) {
            int step = (angle_diff > 0) ? 1 : -1;
            current_angle += step;
            servo_arm.write(current_angle);
            delay(delay_time);
            angle_diff = target_angle - current_angle;
        }
        
        // Ensure final position is reached
        servo_arm.write(target_angle);
        delay(500);  // Wait for servo to settle
        Serial.println("Servo movement complete");
    }

    /**
     * Open the servo to release object
     */
    void open_servo() {
        servo_to_angle(servo_arm_open);
    }
    
    /**
     * Close the servo to grab object
     */
    void close_servo() {
        servo_to_angle(servo_arm_closed);
    }

    /**
     * Get current servo position
     * @return Current angle in degrees
     */
    int get_current_position() {
        return servo_attached ? servo_arm.read() : -1;
    }

    /**
     * Check if servo is attached and ready
     * @return true if servo is properly attached
     */
    bool is_attached() {
        return servo_attached;
    }

    /**
     * Get open position setting
     * @return Open position angle
     */
    int get_open_position() {
        return servo_arm_open;
    }

    /**
     * Get closed position setting
     * @return Closed position angle  
     */
    int get_closed_position() {
        return servo_arm_closed;
    }

    /**
     * Detach servo to save power (stops PWM signal)
     */
    void detach_servo() {
        if (servo_attached) {
            servo_arm.detach();
            servo_attached = false;
            Serial.println("Servo detached");
        }
    }

    /**
     * Re-attach servo after detaching
     */
    void reattach_servo() {
        if (!servo_attached) {
            servo_arm.attach(servo_control_pin, 500, 2400);
            servo_attached = true;
            delay(100);
            Serial.println("Servo reattached");
        }
    }

    /**
     * Test servo movement through full range
     */
    void test_movement() {
        Serial.println("Testing servo movement...");
        
        Serial.println("Moving to center (90°)");
        servo_to_angle(90);
        delay(1000);
        
        Serial.println("Moving to open position");
        open_servo();
        delay(1000);
        
        Serial.println("Moving to closed position");
        close_servo();
        delay(1000);
        
        Serial.println("Returning to open position");
        open_servo();
        
        Serial.println("Servo test complete");
    }
};

#endif // SERVO_CONTROL_H