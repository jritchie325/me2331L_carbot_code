#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"

bool dbug = false;



struct Vec3 {// simple 3D vector class for convenience
  float x{0}, y{0}, z{0};
  Vec3() = default;
  Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
  Vec3 operator+(const Vec3& o) const { return Vec3{x+o.x, y+o.y, z+o.z}; }
  Vec3 operator-(const Vec3& o) const { return Vec3{x-o.x, y-o.y, z-o.z}; }
  Vec3 operator*(float s) const { return Vec3{x*s, y*s, z*s}; }
  Vec3& operator+=(const Vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
};

class ACCELERATE {
public:
  // Public state (read-only via getters)
  const Vec3& gyroRadPerSec() const { return gyro_; }         // x,y,z gyro in rad/s
  const Vec3& accelMps2()     const { return accelLinear_; }  // x,y,z accel in m/s^2 (post-filter, post-gravity)
  const Vec3& velocity()      const { return vel_; }          // integrated m/s (thresholded)
  const Vec3& position()      const { return pos_; }          // integrated m   (thresholded)

  bool begin(uint8_t i2c_addr = 0x68, TwoWire& bus = Wire) {// default I2C address for MPU6050
    bus.begin();                              // start I2C bus
    mpu_ = MPU6050(i2c_addr);                 // default I2C address for MPU6050
    mpu_.initialize();                        // initialize the MPU6050 with default settings
    if (!mpu_.testConnection()) return false; // failed to connect

    mpu_.setDLPFMode(MPU6050_DLPF_BW_20);  // 20 Hz accel/gyro LPF
    mpu_.setRate(4);                       // 1kHz/(1+4)=200 Hz
    mpu_.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu_.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    lastMicros_ = micros(); // for dt calculation
    initialized_ = true;    // successful init
    seedFromAccel();        // seed initial attitude from accel (gravity)
    return true;
  }

  float update() {
    if (!initialized_) return 0.0f;       // not initialized, return 0 dt

    int16_t ax, ay, az, gx, gy, gz;       // raw sensor readings
    mpu_.getAcceleration(&ax, &ay, &az);  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) ... 0x40 (ACCEL_ZOUT_L)
    mpu_.getRotation(&gx, &gy, &gz);      // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L) ... 0x48 (GYRO_ZOUT_L)

    // Time step
    uint32_t nowUs = micros();                                        // current time in microseconds
    float dt = (nowUs >= lastMicros_) ? (nowUs - lastMicros_) * 1e-6f // convert to seconds
                                      : ((UINT32_MAX - lastMicros_ + nowUs) * 1e-6f);
    lastMicros_ = nowUs;
    if (dt <= 0) return 0.0f; // no time elapsed, return 0 dt

    // Constants
    constexpr float LSB_PER_G   = 16384.0f;               // ±2g
    constexpr float G_SI        = 9.80665f;               // standard gravity in m/s^2
    constexpr float LSB_PER_DPS = 131.0f;                 // ±250 dps
    constexpr float DEG2RAD     = 0.017453292519943295f;  // π/180

    // Apply calibrated RAW biases
    float axr = (float)ax - accelBiasRaw_.x;  // raw accel with bias removed
    float ayr = (float)ay - accelBiasRaw_.y;
    float azr = (float)az - accelBiasRaw_.z;

    float gxr = (float)gx - gyroOffset_.x;    // raw gyro with bias removed
    float gyr = (float)gy - gyroOffset_.y;
    float gzr = (float)gz - gyroOffset_.z;

    // Scale to SI
    Vec3 a_si{ (axr/LSB_PER_G)*G_SI, (ayr/LSB_PER_G)*G_SI, (azr/LSB_PER_G)*G_SI };                // accel in m/s^2
    gyro_ = { (gxr/LSB_PER_DPS)*DEG2RAD, (gyr/LSB_PER_DPS)*DEG2RAD, (gzr/LSB_PER_DPS)*DEG2RAD };  // gyro in rad/s

    // ---------- Attitude update with PI accel feedback (Mahony-like) ----------
    // Measured accel direction (unit)
    float axg = a_si.x / G_SI, ayg = a_si.y / G_SI, azg = a_si.z / G_SI;  // normalize to 1g
    float amag_g = sqrtf(axg*axg + ayg*ayg + azg*azg) + 1e-9f;            // avoid divide-by-zero
    float axu = axg / amag_g, ayu = ayg / amag_g, azu = azg / amag_g;     // unit vector of accel direction

    // Expected gravity direction from current attitude (unit): g_hat = [sp, -sr*cp, cr*cp]
    float cr1 = cosf(roll_),  sr1 = sinf(roll_);  // roll is rotation around x-axis
    float cp1 = cosf(pitch_), sp1 = sinf(pitch_); // pitch is rotation around y-axis
    float gxhat = sp1;                            // gravity x in sensor frame
    float gyhat = -sr1 * cp1;                     // gravity y in sensor frame
    float gzhat =  cr1 * cp1;                     // gravity z in sensor frame

    // Accel trust (1 at ~1g, →0 when |a| deviates >10%)
    float accel_trust = 0.0f;                             // default to no trust
    if (amag_g > 0.90f && amag_g < 1.10f) {               // only trust accel when it's close to 1g
      accel_trust = 1.0f - fabsf(amag_g - 1.0f) / 0.10f;  // linear drop-off from 1.0 at 1g to 0.0 at 0.9g or 1.1g
    }

    // Correct error sign: e = g_hat × a_hat
    float ex = (gyhat*azu - gzhat*ayu); // positive correction means we are rolling +x (right wing down), so increase left PWM, decrease right PWM to counter it.
    float ey = (gzhat*axu - gxhat*azu); // positive correction means we are pitching +y (nose up), so increase both PWMs to counter it.
    float ez = (gxhat*ayu - gyhat*axu); // positive correction means we are yawing +z (clockwise), so reduce right PWM, increase left PWM to counter it.

    // PI feedback into gyro; bias integrates in rad/s
    float Kp = Kp_ * accel_trust;   // Proportional gain on accel error (tune)
    float Ki = Ki_ * accel_trust;   // Integral gain on accel error (tune)
    gyroBiasRad_.x += Ki * ex * dt; // integrate gyro bias in rad/s
    gyroBiasRad_.y += Ki * ey * dt;
    gyroBiasRad_.z += Ki * ez * dt;

    auto clamp = [](float v, float m){ return v > m ? m : (v < -m ? -m : v); }; // clamp function to limit values
    gyroBiasRad_.x = clamp(gyroBiasRad_.x, 0.2f); // limit gyro bias to prevent runaway
    gyroBiasRad_.y = clamp(gyroBiasRad_.y, 0.2f);
    gyroBiasRad_.z = clamp(gyroBiasRad_.z, 0.2f);

    // Corrected gyro (rad/s)
    float gx_corr = gyro_.x + Kp * ex - gyroBiasRad_.x; // proportional correction on gyro with bias removed
    float gy_corr = gyro_.y + Kp * ey - gyroBiasRad_.y;
    float gz_corr = gyro_.z + Kp * ez - gyroBiasRad_.z;

    // Integrate attitude
    roll_  += gx_corr * dt; // roll is rotation around x-axis
    pitch_ += gy_corr * dt; // pitch is rotation around y-axis
    yaw_   += gz_corr * dt; // yaw is rotation around z-axis

    // Optional attitude debug
    static uint32_t dbg2 = 0;
    if (millis() - dbg2 > 500 && dbug) {
      dbg2 = millis();
      Serial.print("att[deg]: R="); Serial.print(roll_*180.0/PI);
      Serial.print(" P="); Serial.print(pitch_*180.0/PI);
      Serial.print(" Y="); Serial.println(yaw_*180.0/PI);
    }

    // ---------- Gravity vector in sensor frame (mount-zero corrected) ----------
    float rc = roll_  - roll0_;                     // roll0_ is the roll at calibration, so rc is the current roll relative to that
    float pc = pitch_ - pitch0_;                    // pitch0_ is the pitch at calibration, so pc is the current pitch relative to that
    float cr = cosf(rc),  sr = sinf(rc);            // cr is the cosine of the roll, sr is the sine of the roll
    float cp = cosf(pc),  sp = sinf(pc);            // cp is the cosine of the pitch, sp is the sine of the pitch
    Vec3 g_s{ G_SI*sp, -G_SI*sr*cp, G_SI*cr*cp };   // gravity vector in sensor frame (mount-zero corrected)

    // Debug raw vs gravity
    static uint32_t dbg_ms = 0;
    if ((millis() - dbg_ms > 250) && dbug) {
      dbg_ms = millis();
      Serial.print("rawSI: "); Serial.print(a_si.x); Serial.print(", "); Serial.print(a_si.y); Serial.print(", "); Serial.print(a_si.z);
      Serial.print("   g_s: "); Serial.print(g_s.x); Serial.print(", "); Serial.print(g_s.y); Serial.print(", "); Serial.println(g_s.z);
    }

    // ---------- Linear acceleration (gravity-compensated) ----------
    Vec3 a_lin = a_si - g_s; // linear acceleration in m/s^2 (gravity-compensated)

    // Deadband small noise
    auto db = [](float x, float d){ return (fabsf(x) < d) ? 0.0f : x; }; // deadband function to zero out small values
    a_lin.x = db(a_lin.x, 0.1f);
    a_lin.y = db(a_lin.y, 0.1f);
    a_lin.z = db(a_lin.z, 0.1f);

    // Low-pass (pre-bias)
    Vec3 a_lin_filt = accelLinear_ * (1.0f - accelAlpha_) + a_lin * accelAlpha_;

    // Stillness detection (gyro small + |a|≈1g)
    float gyronorm  = sqrtf(gyro_.x*gyro_.x + gyro_.y*gyro_.y + gyro_.z*gyro_.z);
    bool likely_still = (gyronorm < stillGyroThresh_) && (amag_g > 0.98f && amag_g < 1.02f);

    // Learn linear-accel bias only when still
    if (likely_still) {
      float mu = aLinBiasLearn_ * dt;
      aLinBias_.x += mu * a_lin_filt.x;
      aLinBias_.y += mu * a_lin_filt.y;
      aLinBias_.z += mu * a_lin_filt.z;
      aLinBias_.x = constrain(aLinBias_.x, -aLinBiasMax_, aLinBiasMax_);
      aLinBias_.y = constrain(aLinBias_.y, -aLinBiasMax_, aLinBiasMax_);
      aLinBias_.z = constrain(aLinBias_.z, -aLinBiasMax_, aLinBiasMax_);
    }

    // Corrected accel (what accelMps2() returns)
    Vec3 a_corr{
      a_lin_filt.x - aLinBias_.x,
      a_lin_filt.y - aLinBias_.y,
      a_lin_filt.z - aLinBias_.z
    };
    accelLinear_ = a_corr;

    // Gentle velocity bleed while still
    if (likely_still) vel_ = vel_ * 0.985f;

    // ---------- Integrate velocity/position (ACCEL hysteresis) ----------
    // Hysteresis thresholds on acceleration (tune)
    constexpr float A_ON  = 0.50f; // m/s^2 apply when |accel bucket| >= A_ON     <-----
    constexpr float A_OFF = 0.10f; // m/s^2 leave +/-A_OFF after applying         <-----

    // Accumulate measured (bias-corrected, LPF'd) accel into buckets,
    // and release with hysteresis to form the accel we actually integrate.
    auto applyAccelHyst = [](float meas_a, float &bucket, float on, float off) {
      bucket += meas_a;                      // collect precise accel
      float ab = fabsf(bucket);
      if (ab >= on) {
        float remainder = copysignf(off, bucket);
        float a_release = bucket - remainder; // accel portion to use this step
        bucket = remainder;                   // keep small signed remainder
        return a_release;
      }
      return 0.0f; // below ON: do not pass anything through this step
    };

    // Optional very-slow leak so buckets don’t hold forever if hovering below ON
    float leak = expf(-dt * 0.2f); // ~5 s time constant; comment out to disable
    aBucket_.x *= leak; aBucket_.y *= leak; aBucket_.z *= leak;

    // Build the accel we will integrate this step (a_use)
    Vec3 a_use{
      applyAccelHyst(accelLinear_.x, aBucket_.x, A_ON, A_OFF),
      applyAccelHyst(accelLinear_.y, aBucket_.y, A_ON, A_OFF),
      applyAccelHyst(accelLinear_.z, aBucket_.z, A_ON, A_OFF)
    };

    // Trapezoid integration using the *gated* accel (a_use)
    Vec3 v_prev = vel_;
    Vec3 dv     = (accelPrevUse_ + a_use) * (0.5f * dt);
    vel_       += dv;

    Vec3 dp     = (v_prev + vel_) * (0.5f * dt);
    pos_       += dp;

    // Store for next step
    accelPrevUse_ = a_use;
    accelPrev_    = accelLinear_; // optional: keep for logging/diagnostics

    return dt;
  }

  void calibrate(size_t samples = 1000, bool showProgress = false) {
    long axS=0, ayS=0, azS=0, gxS=0, gyS=0, gzS=0;
    for (size_t i = 0; i < samples; ++i) {
      int16_t ax, ay, az, gx, gy, gz;
      mpu_.getAcceleration(&ax, &ay, &az);
      mpu_.getRotation(&gx, &gy, &gz);
      axS+=ax; ayS+=ay; azS+=az;
      gxS+=gx; gyS+=gy; gzS+=gz;
      if (showProgress && (i % (samples/10 + 1) == 0)) delay(1);
      delayMicroseconds(500);
    }

    // Mean raw (includes gravity)
    float axm = (float)axS / samples;
    float aym = (float)ayS / samples;
    float azm = (float)azS / samples;

    // Estimate roll/pitch at calibration from mean accel (in g)
    constexpr float LSB_PER_G = 16384.0f;
    float axg = axm / LSB_PER_G;
    float ayg = aym / LSB_PER_G;
    float azg = azm / LSB_PER_G;
    float roll_cal  = atan2f(ayg, azg);
    float pitch_cal = -atan2f(axg, sqrtf(ayg*ayg + azg*azg));
    roll0_  = roll_cal;
    pitch0_ = pitch_cal;

    // Gravity in sensor frame at calibration, in RAW LSB
    float cr = cosf(roll_cal),  sr = sinf(roll_cal);
    float cp = cosf(pitch_cal), sp = sinf(pitch_cal);
    Vec3 g_raw{ LSB_PER_G * sp, -LSB_PER_G * sr * cp, LSB_PER_G * cr * cp };

    // Pure sensor bias in raw LSB (mean - gravity_at_cal_pose)
    accelBiasRaw_ = { axm - g_raw.x, aym - g_raw.y, azm - g_raw.z };

    // Gyro bias is just the mean (should be near 0 dps at rest)
    gyroOffset_  = { (float)gxS/samples, (float)gyS/samples, (float)gzS/samples };

    resetDynamics();
    roll_  = roll_cal;
    pitch_ = pitch_cal;
    yaw_   = 0.0f;
  }

  void resetDynamics() {
    vel_ = Vec3{};
    pos_ = Vec3{};
    aBucket_ = Vec3{};
    accelPrevUse_ = Vec3{};
    accelPrev_ = accelLinear_;   // keep if you still want this for debug
    lastMicros_ = micros();
  }

  void resetAll() {
    accelBiasRaw_ = Vec3{};
    gyroOffset_   = Vec3{};
    accelLinear_  = Vec3{};
    accelPrev_    = Vec3{};
    aBucket_      = Vec3{};
    accelPrevUse_ = Vec3{};
    vel_ = Vec3{};
    pos_ = Vec3{};
    roll_ = pitch_ = yaw_ = 0.0f;
    lastMicros_ = micros();
  }

  void setAccelFilterAlpha(float a){ accelAlpha_ = constrain(a,0.0f,1.0f); }

private:
  MPU6050 mpu_{};
  bool initialized_{false};

  float roll0_ = 0.0f, pitch0_ = 0.0f;

  // Raw offsets (LSB)
  Vec3 accelBiasRaw_{};
  Vec3 gyroOffset_{};

  // Attitude estimate
  float roll_{0}, pitch_{0}, yaw_{0};

  // State (SI units)
  Vec3 gyro_{};
  Vec3 accelLinear_{};
  Vec3 accelPrev_{};
  Vec3 vel_{};
  Vec3 pos_{};

  // Mahony-like PI
  float Kp_ = 0.8f;
  float Ki_ = 0.02f;
  Vec3  gyroBiasRad_{0,0,0};

  // Linear accel bias learner
  Vec3  aLinBias_{0,0,0};
  float aLinBiasLearn_ = 0.12f;
  float aLinBiasMax_   = 0.6f;

  // Filtering
  float accelAlpha_{0.08f};

  // Timing
  uint32_t lastMicros_{0};

  // Stillness thresholds
  float stillAccThresh_  = 0.08f; // (kept for reference; not used directly here)
  float stillGyroThresh_ = 0.05f; // rad/s

  Vec3 aBucket_{0,0,0};      // pending accel (m/s^2) to be released with hysteresis
  Vec3 accelPrevUse_{0,0,0}; // last used (gated) accel for trapezoid integration


  void seedFromAccel(){
    int16_t ax,ay,az; int16_t gx,gy,gz;
    mpu_.getAcceleration(&ax,&ay,&az);
    (void)gx;(void)gy;(void)gz;
    constexpr float LSB_PER_G = 16384.0f;
    float axg = ((float)ax)/LSB_PER_G;
    float ayg = ((float)ay)/LSB_PER_G;
    float azg = ((float)az)/LSB_PER_G;
    roll_  = atan2f(ayg, azg);
    pitch_ = -atan2f(axg, sqrtf(ayg*ayg + azg*azg));
    yaw_   = 0.0f;
  }
};

// Alias for compatibility
typedef ACCELERATE Mpu6050Tracker;

//script to test with

/*----------
Mpu6050Tracker imu;

void setup() {
  Serial.begin(115200);
  Serial.println("-------start--------");
  if (!imu.begin()) {
    Serial.println("MPU6050 connection failed!");
    while (1) {}
  }
  delay(500);
  Serial.println(" ---- accelerometer connection initialized ----");

  delay(500);
  Serial.println(" ---- begin calibration ----");
  imu.calibrate(500); // keep sensor still during this
  Serial.println(" ---- Calibration complete ---- ");

  uint32_t tstart = millis();
  while (millis() - tstart < 1500) {
    imu.update();
    delay(5);
  }
}

void loop() {
  static uint32_t lastUpdate = 0, lastPrint = 0;
  uint32_t now = micros();

  // ~200 Hz update
  if ((now - lastUpdate) >= 5000) {
    lastUpdate = now;
    imu.update();
  }

  // ~10 Hz print
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    const Vec3& a = imu.accelMps2();
    const Vec3& v = imu.velocity();
    const Vec3& p = imu.position();
    Serial.print("a [m/s^2]: "); Serial.print(a.x); Serial.print(", "); Serial.print(a.y); Serial.print(", "); Serial.println(a.z);
    Serial.print("v [m/s]:   "); Serial.print(v.x); Serial.print(", "); Serial.print(v.y); Serial.print(", "); Serial.println(v.z);
    Serial.print("p [m]:     "); Serial.print(p.x); Serial.print(", "); Serial.print(p.y); Serial.print(", "); Serial.println(p.z);
    Serial.println();
  }
}
-----------*/
