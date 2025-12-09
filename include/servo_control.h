#include <Servo.h>
#include <Arduino.h>

class SERVO_CONTROL {
private:
    Servo servo_arm; // Create a servo object to control the arm
    int servo_arm_open = 140;   //<----- Servo angle for open position <-----
    int servo_arm_closed = 44;  //<----- Servo angle for closed position <-----
    int servo_speed = 25;       //<----- inverse of speed: higher is slower <-----
    uint8_t servo_control_pin;
    bool servo_attached = false;

public:
    SERVO_CONTROL(uint8_t servo_pin) : servo_control_pin(servo_pin) {
        // Attach servo with proper pulse width limits for better compatibility
        servo_arm.attach(servo_control_pin, 500, 2400);
        servo_attached = true;
        delay(100); // Brief delay for servo to initialize
        Serial.print("Servo attached to pin: ");
        Serial.println(servo_control_pin);
    }

    void speed(int speed_value) {
        servo_speed = speed_value;
    }

    void set_open_value(int open_angle) {
        servo_arm_open = open_angle;
    }

    void set_close_value(int close_angle) {
        servo_arm_closed = close_angle;
    }

    void servo_to_angle(int target_angle) {
        if (!servo_attached) {
            Serial.println("Error: Servo not attached!");
            return;
        }
        
        Serial.print("Moving servo to angle: ");
        Serial.println(target_angle);
        
        int delay_time = 15; // Faster movement
        int current_angle = servo_arm.read();
        int angle_diff = target_angle - current_angle;
        
        Serial.print("Current angle: ");
        Serial.print(current_angle);
        Serial.print(" -> Target: ");
        Serial.println(target_angle);

        // Move servo gradually
        while(abs(angle_diff) > 1) {
            int step = (angle_diff > 0) ? 1 : -1;
            current_angle += step;
            servo_arm.write(current_angle);
            delay(delay_time);
            angle_diff = target_angle - current_angle;
        }
        
        // Final position
        servo_arm.write(target_angle);
        delay(500); // Wait for servo to reach position
        Serial.println("Servo movement complete");
    }

    void open_servo() {
        servo_to_angle(servo_arm_open); // Open the servo to release the object
    }
    void close_servo() {
        servo_to_angle(servo_arm_closed); // Grab the object
    }
};