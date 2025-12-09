/*
  ===============================================================================
  move_PID.h - Advanced Robot Motion Control with PID
  ===============================================================================
  
  Comprehensive robot motion control system combining IMU feedback, PID control,
  and dual H-bridge motor drivers for precise movement and turning.
  
  FEATURES:
  - Two-stage turning system (coarse PID + fine proportional control)
  - IMU-based straight-line correction
  - Time-based and heading-based movement functions
  - Motor balance compensation and minimum power thresholds
  - Debug output control
  
  HARDWARE REQUIREMENTS:
  - Dual H-bridge motor driver (TB6612FNG or similar)
  - MPU6050 IMU for feedback
  - Two DC motors
  
  PIN CONNECTIONS (customizable via constructor):
  - PWM_L:   Pin 6  (Left motor speed)
  - PWM_R:   Pin 5  (Right motor speed)
  - DIR_L:   Pin 8  (Left motor direction)
  - DIR_R:   Pin 7  (Right motor direction)
  - STANDBY: Pin 3  (Motor driver enable)
  
  DEPENDENCIES:
  - PID_v1 library (Brett Beauregard)
  - accelerate.h (MPU6050 wrapper)
  
  USAGE:
    RobotCarPID robot(6, 5, 8, 7, 3);  // Custom pins
    robot.begin();
    robot.moveForTime(0.5, 2000);      // 0.5 m/s for 2 seconds
    robot.turnToHeading(90);            // Turn to 90 degrees
  
  ===============================================================================
*/

#ifndef MOVE_PID_H
#define MOVE_PID_H

#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include "accelerate.h"

class RobotCarPID {
private:
    // ==================== PIN CONFIGURATION ====================
    uint8_t PWM_L_PIN   = 6;   // PWM pin for left motor speed control
    uint8_t PWM_R_PIN   = 5;   // PWM pin for right motor speed control  
    uint8_t DIR_L_PIN   = 8;   // Digital pin for left motor direction control
    uint8_t DIR_R_PIN   = 7;   // Digital pin for right motor direction control
    uint8_t STANDBY_PIN = 3;   // Digital pin for standby control

    // ==================== IMU OBJECT ====================
    ACCELERATE imu;
    double yaw_rad = 0.0;      // Integrated yaw angle in radians

    // ==================== PID CONTROLLERS ====================
    // Movement PID (speed control)
    double move_setpoint, move_input, move_output;
    double v_proportional = 8.0, v_integral = 0.5, v_delta = 0.3;
    PID speedPID;

    // Turning PID (heading control)
    double turn_setpoint, turn_input, turn_output;
    double t_proportional = 6, t_integral = 0.5, t_delta = 1.1;
    PID turnPID;

    // ==================== CONTROL PARAMETERS ====================
    enum class ControlMode { MOVE_STRAIGHT, TURN_IN_PLACE };
    ControlMode mode = ControlMode::MOVE_STRAIGHT;

    double target_speed_mps = 0.4;        // Target speed in m/s
    double target_heading_rad = 0.0;      // Target heading in radians

    // Motor limits and tuning parameters
    const int PWM_MAX = 240;              // Maximum PWM value
    const int TURN_MAX = 40;              // Max PWM for fine turning
    const int TURN_SPIN_MAX = 220;        // Max PWM for coarse turning
    const double HEADING_TOL = 1.5 * (M_PI / 180.0);  // Final heading tolerance

    uint32_t lastPrintMs = 0;
    bool debug_enabled = false;           // Debug output control

    // ==================== UTILITY FUNCTIONS ====================
    int clampi(int v, int lo, int hi) { 
        return (v < lo) ? lo : ((v > hi) ? hi : v); 
    }
    
    double constrainf(double v, double lo, double hi) { 
        return (v < lo) ? lo : ((v > hi) ? hi : v); 
    }

    /**
     * Drive motors with signed PWM values (-255 to +255)
     * Includes minimum power thresholds and motor balance compensation
     */
    void driveSigned(int left, int right) {
        left  = clampi(left,  -PWM_MAX, PWM_MAX);
        right = clampi(right, -PWM_MAX, PWM_MAX);
        
        // Apply minimum power to overcome static friction
        const int MIN_MOTOR_POWER = 50;
        
        if (left != 0) {
            if (abs(left) < MIN_MOTOR_POWER) {
                left = (left > 0) ? MIN_MOTOR_POWER : -MIN_MOTOR_POWER;
            }
        }
        if (right != 0) {
            if (abs(right) < MIN_MOTOR_POWER) {
                right = (right > 0) ? MIN_MOTOR_POWER : -MIN_MOTOR_POWER;
            }
        }

        // Motor balance compensation (adjust if needed)
        const float LEFT_MOTOR_COMPENSATION = 1.0;
        const float RIGHT_MOTOR_COMPENSATION = 1.0;
        
        left = (int)(left * LEFT_MOTOR_COMPENSATION);
        right = (int)(right * RIGHT_MOTOR_COMPENSATION);
        
        left = clampi(left, -PWM_MAX, PWM_MAX);
        right = clampi(right, -PWM_MAX, PWM_MAX);
        
        // Set motor directions and speeds
        digitalWrite(DIR_L_PIN, left >= 0 ? HIGH : LOW);
        analogWrite(PWM_L_PIN, abs(left));
        digitalWrite(DIR_R_PIN, right >= 0 ? HIGH : LOW);
        analogWrite(PWM_R_PIN, abs(right));

        // Enable motor driver when active
        bool active = (left != 0) || (right != 0);
        digitalWrite(STANDBY_PIN, active ? HIGH : LOW);
        
        // Debug output
        if (debug_enabled && active) {
            static uint32_t lastDebug = 0;
            if (millis() - lastDebug > 300) {
                lastDebug = millis();
                Serial.print("Motors: L="); Serial.print(left);
                Serial.print(", R="); Serial.print(right);
                Serial.print(", Active="); Serial.println(active);
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
            Serial.print("Speed: "); Serial.print(v_forward, 3);
            Serial.print(" m/s, Yaw: "); Serial.print(yaw_rad * 180.0 / M_PI, 1);
            Serial.print("°, Mode: "); 
            Serial.println(mode == ControlMode::MOVE_STRAIGHT ? "MOVE" : "TURN");
        }
    }

public:
    // ==================== CONSTRUCTORS ====================
    /**
     * Default constructor with standard pins
     */
    RobotCarPID() : speedPID(&move_input, &move_output, &move_setpoint, v_proportional, v_integral, v_delta, DIRECT),
                    turnPID(&turn_input, &turn_output, &turn_setpoint, t_proportional, t_integral, t_delta, DIRECT) {
    }
    
    /**
     * Constructor with custom pin configuration
     */
    RobotCarPID(uint8_t pwm_l, uint8_t pwm_r, uint8_t dir_l, uint8_t dir_r, uint8_t standby) 
        : PWM_L_PIN(pwm_l), PWM_R_PIN(pwm_r), DIR_L_PIN(dir_l), DIR_R_PIN(dir_r), STANDBY_PIN(standby),
          speedPID(&move_input, &move_output, &move_setpoint, v_proportional, v_integral, v_delta, DIRECT),
          turnPID(&turn_input, &turn_output, &turn_setpoint, t_proportional, t_integral, t_delta, DIRECT) {
    }

    // ==================== DEBUG CONTROL ====================
    void enableDebug(bool enable = true) { debug_enabled = enable; }
    void disableDebug() { debug_enabled = false; }
    bool isDebugEnabled() const { return debug_enabled; }

    // ==================== INITIALIZATION ====================
    /**
     * Initialize hardware and IMU
     * @return true if successful
     */
    bool begin() {
        // Configure motor pins
        pinMode(PWM_L_PIN, OUTPUT);
        pinMode(PWM_R_PIN, OUTPUT);
        pinMode(DIR_L_PIN, OUTPUT);
        pinMode(DIR_R_PIN, OUTPUT);
        pinMode(STANDBY_PIN, OUTPUT);
        stopAll();

        // Initialize IMU
        if (!imu.begin()) {
            if (debug_enabled) {
                Serial.println("MPU6050 connection failed");
            }
            return false;
        }
        delay(400);
        imu.calibrate(1500);  // Calibrate IMU

        // Reset heading
        target_heading_rad = 0.0;
        yaw_rad = 0.0;

        // Configure PID controllers
        speedPID.SetOutputLimits(0, PWM_MAX);
        speedPID.SetSampleTime(10);
        speedPID.SetMode(AUTOMATIC);

        turnPID.SetSampleTime(10);
        turnPID.SetOutputLimits(-TURN_MAX, TURN_MAX);
        turnPID.SetMode(AUTOMATIC);

        return true;
    }

    // ==================== MOVEMENT FUNCTIONS ====================
    
    /**
     * Move at specified speed for specified time
     * @param speed_mps Speed in meters per second
     * @param time_ms Duration in milliseconds
     * @param enableTelemetry Print debug information
     */
    void moveForTime(double speed_mps, unsigned long time_ms, bool enableTelemetry = false) {
        target_speed_mps = speed_mps;
        mode = ControlMode::MOVE_STRAIGHT;
        
        // Reset for clean start
        stopAll();
        delay(100);
        resetPIDControllers();
        imu.resetDynamics();
        yaw_rad = 0.0;
        
        unsigned long start_time = millis();
        
        while (millis() - start_time < time_ms) {
            updateIMU();
            
            double v_forward = imu.velocity().x;
            
            // Speed control
            move_setpoint = target_speed_mps;
            move_input = v_forward;
            speedPID.Compute();
            int basePWM = (int)move_output;

            // Gentle heading correction for straight movement
            const int STRAIGHT_TURN_MAX = 12;
            turn_setpoint = 0.0;
            turn_input = imu.gyroRadPerSec().z;
            turnPID.SetOutputLimits(-STRAIGHT_TURN_MAX, STRAIGHT_TURN_MAX);
            turnPID.SetTunings(1.5, 0.1, 0.2);  // Gentle correction
            turnPID.Compute();
            int corr = (int)turn_output;

            // Apply motor commands
            int leftCmd = basePWM + corr;
            int rightCmd = basePWM - corr;
            driveSigned(leftCmd, rightCmd);
            
            if (enableTelemetry) printTelemetry();
            delay(20);
        }
        
        stopAll();
        turnPID.SetTunings(t_proportional, t_integral, t_delta);  // Restore turn PID
        imu.resetDynamics();
    }

    /**
     * Turn to specified heading using two-stage control system
     * @param heading_deg Target heading in degrees
     * @param enableTelemetry Print debug information
     */
    void turnToHeading(double heading_deg, bool enableTelemetry = false) {
        // Reset for clean start
        stopAll();
        delay(100);
        resetPIDControllers();
        imu.resetDynamics();
        yaw_rad = 0.0;
        
        target_heading_rad = heading_deg * M_PI / 180.0;
        mode = ControlMode::TURN_IN_PLACE;
        
        // Two-stage control thresholds
        const double FINE_CONTROL_THRESHOLD = 12.0 * (M_PI / 180.0);  // 12 degrees
        const double FINAL_TOLERANCE = 2.5 * (M_PI / 180.0);          // 2.5 degrees
        
        unsigned long target_hold_start = 0;
        const unsigned long HOLD_TIME_MS = 300;
        bool holding_at_target = false;
        bool in_fine_control = false;
        
        while (true) {
            updateIMU();
            
            double heading_err = target_heading_rad - yaw_rad;
            double abs_heading_err = fabs(heading_err);
            
            int leftCmd = 0, rightCmd = 0;
            
            if (abs_heading_err > FINE_CONTROL_THRESHOLD) {
                // Stage 1: Aggressive PID for coarse turning
                in_fine_control = false;
                turn_setpoint = target_heading_rad;
                turn_input = yaw_rad;
                turnPID.SetOutputLimits(-TURN_SPIN_MAX, TURN_SPIN_MAX);
                turnPID.Compute();
                
                int spin = (int)turn_output;
                leftCmd = -spin;
                rightCmd = +spin;
                
                if (debug_enabled) {
                    Serial.print("Coarse: "); Serial.print(abs_heading_err * 180.0 / M_PI);
                    Serial.print("°, power="); Serial.println(spin);
                }
                
            } else {
                // Stage 2: Gentle proportional control for fine positioning
                if (!in_fine_control) {
                    if (debug_enabled) Serial.println("Fine control mode");
                    in_fine_control = true;
                    resetPIDControllers();
                }
                
                const double MICRO_DEADZONE = 3.0 * (M_PI / 180.0);  // 3 degrees
                const int MAX_FINE_POWER = 65;
                const int MIN_FINE_POWER = 35;
                
                if (abs_heading_err > MICRO_DEADZONE) {
                    double power_ratio = (abs_heading_err - MICRO_DEADZONE) / 
                                       (FINE_CONTROL_THRESHOLD - MICRO_DEADZONE);
                    power_ratio = constrainf(power_ratio, 0.0, 1.0);
                    int fine_power = MIN_FINE_POWER + 
                                   (int)((MAX_FINE_POWER - MIN_FINE_POWER) * power_ratio);
                    
                    if (heading_err > 0) {  // Turn left
                        leftCmd = -fine_power;
                        rightCmd = +fine_power;
                    } else {  // Turn right
                        leftCmd = +fine_power;
                        rightCmd = -fine_power;
                    }
                    
                    if (debug_enabled) {
                        Serial.print("Fine: "); Serial.print(abs_heading_err * 180.0 / M_PI);
                        Serial.print("°, power="); Serial.println(fine_power);
                    }
                }
            }
            
            // Check if target reached
            if (abs_heading_err < FINAL_TOLERANCE) {
                if (!holding_at_target) {
                    holding_at_target = true;
                    target_hold_start = millis();
                    if (debug_enabled) Serial.println("Target reached, holding...");
                }
                
                if (millis() - target_hold_start > HOLD_TIME_MS) {
                    if (debug_enabled) Serial.println("Turn complete");
                    break;
                }
            } else {
                holding_at_target = false;
            }

            driveSigned(leftCmd, rightCmd);
            if (enableTelemetry) printTelemetry();
            delay(10);
        }
        
        stopAll();
        imu.resetDynamics();
    }

    // ==================== UTILITY METHODS ====================
    
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
        speedPID.SetMode(MANUAL);
        turnPID.SetMode(MANUAL);
        move_output = 0;
        turn_output = 0;
        speedPID.SetMode(AUTOMATIC);
        turnPID.SetMode(AUTOMATIC);
    }

    // ==================== SENSOR READINGS ====================
    
    double getCurrentSpeed() { return imu.velocity().x; }
    double getCurrentHeading() { return yaw_rad * 180.0 / M_PI; }
    double getDistanceTraveled() { return imu.position().x; }

    // ==================== PID TUNING ====================
    
    void setSpeedPIDValues(double kp, double ki, double kd) {
        v_proportional = kp; v_integral = ki; v_delta = kd;
        speedPID.SetTunings(kp, ki, kd);
    }

    void setTurnPIDValues(double kp, double ki, double kd) {
        t_proportional = kp; t_integral = ki; t_delta = kd;
        turnPID.SetTunings(kp, ki, kd);
    }

    /**
     * Simple motor test function for hardware verification
     */
    void testMotors(int pwm_value = 80, unsigned long test_time_ms = 2000) {
        Serial.println("Testing motors...");
        
        Serial.println("Forward test");
        driveSigned(pwm_value, pwm_value);
        delay(test_time_ms);
        stopAll();
        delay(1000);
        
        Serial.println("Backward test");
        driveSigned(-pwm_value, -pwm_value);
        delay(test_time_ms);
        stopAll();
        delay(1000);
        
        Serial.println("Turn left test");
        driveSigned(-pwm_value, pwm_value);
        delay(test_time_ms);
        stopAll();
        delay(1000);
        
        Serial.println("Turn right test");
        driveSigned(pwm_value, -pwm_value);
        delay(test_time_ms);
        stopAll();
        
        Serial.println("Motor test complete");
    }
};

#endif // MOVE_PID_H