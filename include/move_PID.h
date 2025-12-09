/*
  RobotCar_PID_MPU6050 Class

  - Libraries:
      Wire.h
      MPU6050 + I2Cdev (Jeff Rowberg)
      PID_v1.h
  - Hardware:
      Dual H-bridge with 5 signals:
        PWM_L (0-255), DIR_L (HIGH fwd, LOW rev)
        PWM_R (0-255), DIR_R (HIGH fwd, LOW rev)
        STANDBY (enable HIGH)
  - IMU:
      Uses Mpu6050Tracker from earlier. Velocity x is treated as forward speed.
      Yaw is integrated from gyro z.

  Notes:
    - IMU velocity and heading will drift; tune PIDs conservatively and consider adding wheel encoders for long runs.
*/

#ifndef MOVE_PID_H
#define MOVE_PID_H

#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include "accelerate.h"

class RobotCarPID {
private:
    // -------------------- Pin mapping --------------------
    uint8_t PWM_L_PIN   = 6;   // PWM pin for left motor speed control
    uint8_t PWM_R_PIN   = 5;   // PWM pin for right motor speed control  
    uint8_t DIR_L_PIN   = 8;   // Digital pin for left motor direction control
    uint8_t DIR_R_PIN   = 7;   // Digital pin for right motor direction control
    uint8_t STANDBY_PIN = 3;   // Digital pin for standby control

    // -------------------- Motion/IMU objects --------------------
    ACCELERATE imu;

    // Integrated yaw angle (rad), using gyro z
    double yaw_rad = 0.0;

    // -------------------- PID variables & objects --------
    double move_setpoint, move_input, move_output;            // movement PID
    double v_proportional = 8.0, v_integral = 0.5, v_delta = 0.3;   // Much more aggressive for faster response
    PID speedPID;

    double turn_setpoint, turn_input, turn_output;            // turning PID
    double t_proportional = 6, t_integral = 0.5, t_delta = 1.1;   // Aggressive for turning, gentle for straight-line corrections
    PID turnPID;

    // -------------------- User targets & mode --------------------
    enum class ControlMode { MOVE_STRAIGHT, TURN_IN_PLACE };
    ControlMode mode = ControlMode::MOVE_STRAIGHT;

    // Desired forward speed (m/s) for MOVE_STRAIGHT
    double target_speed_mps = 0.4;      // example: ~0.4 m/s

    // Desired heading (rad) for TURN_IN_PLACE
    double target_heading_rad = 0.0;

    // -------------------- Limits and timing --------------------
    const int PWM_MAX = 240;          // Near maximum PWM (255 is max)
    const int TURN_MAX = 40;         // Reduced for gentler fine control
    const int TURN_SPIN_MAX = 220;    // Nearly full power for turns
    const double HEADING_TOL = 1.5 * (M_PI / 180.0);  // Final tolerance 1.5° for better precision

    uint32_t lastPrintMs = 0;
    bool debug_enabled = false;  //<----- Debug output control flag - false by default <-----

    // -------------------- Private Helper Methods --------------------
    int clampi(int v, int lo, int hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }
    double constrainf(double v, double lo, double hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }

    // Signed drive: negative = reverse, positive = forward
    void driveSigned(int left, int right) {
        left  = clampi(left,  -PWM_MAX, PWM_MAX);
        right = clampi(right, -PWM_MAX, PWM_MAX);
        
        // Add minimum base power to overcome static friction
        const int MIN_MOTOR_POWER = 50;  // Minimum PWM to ensure motors actually move
        
        // Apply minimum power when motors should be active
        if (left != 0) {
            if (abs(left) < MIN_MOTOR_POWER) {
                if(left > 0) {
                    left = MIN_MOTOR_POWER;
                } else {
                    left = -MIN_MOTOR_POWER;
                }
            }
        }
        if (right != 0) {
            if (abs(right) < MIN_MOTOR_POWER) {
                if(right > 0) {
                    right = MIN_MOTOR_POWER;
                } else {
                    right = -MIN_MOTOR_POWER;
                }
            }
        }

        // Motor balance compensation - adjust if one motor is weaker
        // You can fine-tune these multipliers if needed (1.0 = no adjustment)
        const float LEFT_MOTOR_COMPENSATION = 1.0;
        const float RIGHT_MOTOR_COMPENSATION = 1.0;
        
        left = (int)(left * LEFT_MOTOR_COMPENSATION);
        right = (int)(right * RIGHT_MOTOR_COMPENSATION);
        
        // Clamp again after compensation
        left = clampi(left, -PWM_MAX, PWM_MAX);
        right = clampi(right, -PWM_MAX, PWM_MAX);
        
        // Left side
        if (left >= 0) {
            digitalWrite(DIR_L_PIN, HIGH);
            analogWrite(PWM_L_PIN, left);
        } else {
            digitalWrite(DIR_L_PIN, LOW);
            analogWrite(PWM_L_PIN, -left);
        }

        // Right side
        if (right >= 0) {
            digitalWrite(DIR_R_PIN, HIGH);
            analogWrite(PWM_R_PIN, right);
        } else {
            digitalWrite(DIR_R_PIN, LOW);
            analogWrite(PWM_R_PIN, -right);
        }

        // Standby enable whenever any nonzero command is issued
        bool active = (left != 0) || (right != 0);
        digitalWrite(STANDBY_PIN, active ? HIGH : LOW);
        
        // Debug output for troubleshooting motor power
        if (debug_enabled) {
            static uint32_t lastDebug = 0;
            if (active && millis() - lastDebug > 300) {
                lastDebug = millis();
                Serial.print("Motors: L="); Serial.print(left);
                Serial.print(", R="); Serial.print(right);
                Serial.print(", PWM_MAX="); Serial.print(PWM_MAX);
                Serial.print(", STANDBY="); Serial.println(active ? "HIGH" : "LOW");
            }
        }
    }

    void stopAll() {
        analogWrite(PWM_L_PIN, 0);
        analogWrite(PWM_R_PIN, 0);
        digitalWrite(STANDBY_PIN, LOW);
    }

    void updateIMU() {
        float dt = imu.update();
        if (dt > 0) {
            yaw_rad += imu.gyroRadPerSec().z * dt;
        }
    }

    void printTelemetry() {
        if (millis() - lastPrintMs > 100) {
            lastPrintMs = millis();
            double v_forward = imu.velocity().x;
            Serial.print("v_fwd[m/s]="); Serial.print(v_forward, 3);
            Serial.print("  yaw[deg]=");  Serial.print(yaw_rad * 180.0 / M_PI, 1);
            Serial.print("  move_out=");  Serial.print(move_output, 0);
            Serial.print("  turn_out=");  Serial.print(turn_output, 0);
            Serial.print("  mode=");      Serial.println(mode == ControlMode::MOVE_STRAIGHT ? "MOVE" : "TURN");
        }
    }

public:
    // -------------------- Constructors --------------------
    // Default constructor with standard pins
    RobotCarPID() : speedPID(&move_input, &move_output, &move_setpoint, v_proportional, v_integral, v_delta, DIRECT),
                    turnPID(&turn_input, &turn_output, &turn_setpoint, t_proportional, t_integral, t_delta, DIRECT) {
    }
    
    // Constructor with custom pin configuration
    RobotCarPID(uint8_t pwm_l, uint8_t pwm_r, uint8_t dir_l, uint8_t dir_r, uint8_t standby) 
        : PWM_L_PIN(pwm_l), PWM_R_PIN(pwm_r), DIR_L_PIN(dir_l), DIR_R_PIN(dir_r), STANDBY_PIN(standby),
          speedPID(&move_input, &move_output, &move_setpoint, v_proportional, v_integral, v_delta, DIRECT),
          turnPID(&turn_input, &turn_output, &turn_setpoint, t_proportional, t_integral, t_delta, DIRECT) {
    }

    // -------------------- Debug Control --------------------
    void enableDebug(bool enable = true) { debug_enabled = enable; }
    void disableDebug() { debug_enabled = false; }
    bool isDebugEnabled() const { return debug_enabled; }

    // -------------------- Initialization --------------------
    bool begin() {
        Serial.begin(115200);
        pinMode(PWM_L_PIN, OUTPUT);
        pinMode(PWM_R_PIN, OUTPUT);
        pinMode(DIR_L_PIN, OUTPUT);
        pinMode(DIR_R_PIN, OUTPUT);
        pinMode(STANDBY_PIN, OUTPUT);
        stopAll();

        if (!imu.begin()) {
            if (debug_enabled) {
                Serial.println("MPU6050 failed to connect. Halt.");
            }
            return false;
        }
        delay(400);
        imu.calibrate(1500);

        // Initialize heading setpoint to current yaw
        target_heading_rad = 0.0;
        yaw_rad = 0.0;

        // PID setup with faster sampling and higher limits
        speedPID.SetOutputLimits(0, PWM_MAX);
        speedPID.SetSampleTime(10);  // Faster sampling for quicker response
        speedPID.SetMode(AUTOMATIC);

        turnPID.SetSampleTime(10);   // Faster sampling
        turnPID.SetOutputLimits(-TURN_MAX, TURN_MAX);
        turnPID.SetMode(AUTOMATIC);

        return true;
    }

    // -------------------- Movement Control Methods --------------------

    void moveForTime(double speed_mps, unsigned long time_ms, bool enableTelemetry = false) {
        target_speed_mps = speed_mps;
        mode = ControlMode::MOVE_STRAIGHT;
        
        // Reset everything before starting movement
        stopAll();
        delay(100); // Brief pause for motors to stop
        resetPIDControllers();
        imu.resetDynamics();
        yaw_rad = 0.0;
        
        unsigned long start_time = millis();
        
        while (millis() - start_time < time_ms) {
            updateIMU();
            
            double v_forward = imu.velocity().x;
            //bool moving_backward = target_speed_mps < 0;
            
            // Speed PID → base PWM (use signed values for proper feedback)
            move_setpoint = target_speed_mps;  // Use signed setpoint
            move_input = v_forward;            // Use signed velocity feedback
            speedPID.Compute();
            int basePWM = (int)move_output;

            // Gentle heading correction for straight movement (much less aggressive than turning)
            const int STRAIGHT_TURN_MAX = 12; // Much gentler correction to prevent stuttering
            turn_setpoint = 0.0;
            turn_input = imu.gyroRadPerSec().z;
            turnPID.SetOutputLimits(-STRAIGHT_TURN_MAX, STRAIGHT_TURN_MAX);
            // Temporarily use gentler PID settings for straight movement
            turnPID.SetTunings(1.5, 0.1, 0.2); // Much gentler than turning
            turnPID.Compute();
            int corr = (int)turn_output;

            // Compose wheel commands
            int leftCmd = basePWM + corr;
            int rightCmd = basePWM - corr;
            driveSigned(leftCmd, rightCmd);
            
            if (enableTelemetry) {
                printTelemetry();
            }
            
            delay(20); // Control loop timing
        }
        
        stopAll();
        // Restore aggressive turn PID settings for turning operations
        turnPID.SetTunings(t_proportional, t_integral, t_delta);
        imu.resetDynamics(); // Reset dynamics after movement
    }

    void turnToHeading(double heading_deg, bool enableTelemetry = false) {
        // Reset everything before starting turn
        stopAll();
        delay(100); // Brief pause for motors to stop
        resetPIDControllers();
        imu.resetDynamics();
        yaw_rad = 0.0;
        
        target_heading_rad = heading_deg * M_PI / 180.0;
        mode = ControlMode::TURN_IN_PLACE;
        
        const double FINE_CONTROL_THRESHOLD = 12.0 * (M_PI / 180.0); // Switch to fine control at 12°
        const double FINAL_TOLERANCE = 2.5 * (M_PI / 180.0);         // Final target tolerance (2.5°)
        
        unsigned long target_hold_start = 0;
        const unsigned long HOLD_TIME_MS = 300; // Hold at target longer for stability
        bool holding_at_target = false;
        bool in_fine_control = false;
        
        while (true) {
            updateIMU();
            
            double heading_err = target_heading_rad - yaw_rad;
            double abs_heading_err = fabs(heading_err);
            
            int leftCmd = 0, rightCmd = 0;
            
            // Two-stage control system
            if (abs_heading_err > FINE_CONTROL_THRESHOLD) {
                // Stage 1: Aggressive PID for coarse turning (>8°)
                in_fine_control = false;
                turn_setpoint = target_heading_rad;
                turn_input = yaw_rad;
                turnPID.SetOutputLimits(-TURN_SPIN_MAX, TURN_SPIN_MAX);
                turnPID.Compute();
                
                int spin = (int)turn_output;
                leftCmd = -spin;
                rightCmd = +spin;
                
                if (debug_enabled) {
                    Serial.print("Coarse turn: err="); Serial.print(abs_heading_err * 180.0 / M_PI);
                    Serial.print("°, spin="); Serial.println(spin);
                }
                
            } else {
                // Stage 2: Gentle proportional control for fine positioning (≤8°)
                if (!in_fine_control) {
                    if (debug_enabled) {
                        Serial.println("Switching to fine control mode");
                    }
                    in_fine_control = true;
                    // Reset PID to prevent windup from affecting fine control
                    resetPIDControllers();
                }
                
                // Very gentle proportional control with deadzone
                const double MICRO_DEADZONE = 3.0 * (M_PI / 180.0); // Don't move if within 3°
                const int MAX_FINE_POWER = 65;  // Much gentler max power
                const int MIN_FINE_POWER = 35;  // Lower minimum power
                int fine_power = 0; // Initialize for debug output
                
                if (abs_heading_err > MICRO_DEADZONE) {
                    // Only move if error is significant enough
                    // Proportional control: power scales with error
                    double power_ratio = (abs_heading_err - MICRO_DEADZONE) / (FINE_CONTROL_THRESHOLD - MICRO_DEADZONE);
                    power_ratio = constrainf(power_ratio, 0.0, 1.0);
                    fine_power = MIN_FINE_POWER + (int)((MAX_FINE_POWER - MIN_FINE_POWER) * power_ratio);
                    
                    // Direction based on error sign
                    if (heading_err > 0) { // Need to turn counterclockwise (left)
                        leftCmd = -fine_power;
                        rightCmd = +fine_power;
                    } else { // Need to turn clockwise (right)
                        leftCmd = +fine_power;
                        rightCmd = -fine_power;
                    }
                } else {
                    // Within deadzone - stop moving
                    leftCmd = 0;
                    rightCmd = 0;
                    fine_power = 0;
                }
                
                if (debug_enabled) {
                    Serial.print("Fine control: err="); Serial.print(abs_heading_err * 180.0 / M_PI);
                    Serial.print("°, power="); Serial.println(fine_power);
                }
            }
            
            // Check if we're within final tolerance
            if (abs_heading_err < FINAL_TOLERANCE) {
                if (!holding_at_target) {
                    holding_at_target = true;
                    target_hold_start = millis();
                    if (debug_enabled) {
                        Serial.println("Target reached, holding position...");
                    }
                }
                
                if (millis() - target_hold_start > HOLD_TIME_MS) {
                    if (debug_enabled) {
                        Serial.println("Hold period complete");
                    }
                    break;
                }
                
                // Continue with gentle holding force
            } else {
                holding_at_target = false;
            }

            driveSigned(leftCmd, rightCmd);
            
            if (enableTelemetry) {
                printTelemetry();
            }
            
            delay(10); // Control loop timing
        }
        
        stopAll();
        imu.resetDynamics(); // Reset dynamics after turn
    }

    void turnForTime(double turn_rate_deg_per_sec, unsigned long time_ms, bool enableTelemetry = false) {
        // Reset everything before starting turn
        stopAll();
        delay(100); // Brief pause for motors to stop
        resetPIDControllers();
        imu.resetDynamics();
        yaw_rad = 0.0;
        
        // Calculate target heading based on turn rate and time
        double target_angle_deg = turn_rate_deg_per_sec * (time_ms / 1000.0);
        target_heading_rad = target_angle_deg * M_PI / 180.0;
        mode = ControlMode::TURN_IN_PLACE;
        
        unsigned long start_time = millis();
        
        while (millis() - start_time < time_ms) {
            updateIMU();
            
            // Turn PID on heading
            turn_setpoint = target_heading_rad;
            turn_input = yaw_rad;
            turnPID.SetOutputLimits(-TURN_SPIN_MAX, TURN_SPIN_MAX);
            turnPID.Compute();

            int spin = (int)turn_output;
            int leftCmd = -spin;
            int rightCmd = +spin;
            
            // Let PID handle power control directly for smoother response

            driveSigned(leftCmd, rightCmd);
            
            if (enableTelemetry) {
                printTelemetry();
            }
            
            delay(20); // Control loop timing
        }
        
        // Hold at final position briefly
        Serial.println("Turn time complete, holding final position...");
        for (int i = 0; i < 25; i++) { // Hold for 0.5 seconds
            updateIMU();
            turn_setpoint = target_heading_rad;
            turn_input = yaw_rad;
            turnPID.Compute();
            int spin = (int)turn_output;
            // Let deadband handle minimum thresholds for fine control
            driveSigned(-spin, spin);
            delay(20);
        }
        
        stopAll();
        imu.resetDynamics(); // Reset dynamics after turn
    }

    // -------------------- Utility Methods --------------------
    void cancelMotion() {
        target_speed_mps = 0.0;
        mode = ControlMode::MOVE_STRAIGHT;
        stopAll();
        imu.resetDynamics();
    }

    void resetIMU() {
        imu.resetDynamics();
        yaw_rad = 0.0;
    }
    
    void resetPIDControllers() {
        // Reset PID accumulated errors to prevent carryover between modes
        speedPID.SetMode(MANUAL);
        turnPID.SetMode(MANUAL);
        move_output = 0;
        turn_output = 0;
        speedPID.SetMode(AUTOMATIC);
        turnPID.SetMode(AUTOMATIC);
    }

    // Get current sensor readings
    double getCurrentSpeed() {
        return imu.velocity().x;
    }

    double getCurrentHeading() {
        return yaw_rad * 180.0 / M_PI;
    }

    double getDistanceTraveled() {
        return imu.position().x;
    }

    // Simple motor test function
    void testMotors(int pwm_value = 80, unsigned long test_time_ms = 2000) {
        Serial.println("Testing motors...");
        
        // Test forward
        Serial.println("Forward test");
        driveSigned(pwm_value, pwm_value);
        delay(test_time_ms);
        stopAll();
        delay(1000);
        
        // Test backward  
        Serial.println("Backward test");
        driveSigned(-pwm_value, -pwm_value);
        delay(test_time_ms);
        stopAll();
        delay(1000);
        
        // Test turn left
        Serial.println("Turn left test");
        driveSigned(-pwm_value, pwm_value);
        delay(test_time_ms);
        stopAll();
        delay(1000);
        
        // Test turn right
        Serial.println("Turn right test");
        driveSigned(pwm_value, -pwm_value);
        delay(test_time_ms);
        stopAll();
        
        Serial.println("Motor test complete");
    }

    // -------------------- PID Tuning Methods --------------------
    void setSpeedPIDValues(double kp, double ki, double kd) {
        v_proportional = kp;
        v_integral = ki;
        v_delta = kd;
        speedPID.SetTunings(kp, ki, kd);
    }

    void setTurnPIDValues(double kp, double ki, double kd) {
        t_proportional = kp;
        t_integral = ki;
        t_delta = kd;
        turnPID.SetTunings(kp, ki, kd);
    }
};

#endif // MOVE_PID_H



/*
void simpleSquarePattern() {
    // Drive in a square pattern
    for (int i = 0; i < 4; i++) {
        robot.moveForDistance(0.3, 1.0);  // Move 1m forward
        delay(500);
        robot.turnToHeading(i * 90.0);    // Turn 90 degrees
        delay(500);
    }
}

void speedTest() {
    // Test different speeds
    robot.moveForTime(0.1, 2000);  // Slow: 0.1 m/s for 2 seconds
    delay(1000);
    robot.moveForTime(0.3, 2000);  // Medium: 0.3 m/s for 2 seconds
    delay(1000);
    robot.moveForTime(0.5, 2000);  // Fast: 0.5 m/s for 2 seconds
}

void precisionTurning() {
    // Test precise turning
    robot.turnToHeading(45.0);     // Turn to 45 degrees
    delay(1000);
    robot.turnToHeading(-30.0);    // Turn to -30 degrees
    delay(1000);
    robot.turnToHeading(0.0);      // Return to 0 degrees
}
*/