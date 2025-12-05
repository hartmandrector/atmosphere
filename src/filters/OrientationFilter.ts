/**
 * Extended Kalman Filter for IMU orientation estimation
 * State: [q0, q1, q2, q3, ωx, ωy, ωz, bx, by, bz]
 * - q: Unit quaternion representing orientation
 * - ω: Angular velocity (rad/s)
 * - b: Gyroscope bias (rad/s)
 */

import { Quaternion } from '../utils/Quaternion';

export class OrientationFilter {
  // State vector: [q0, q1, q2, q3, ωx, ωy, ωz, bx, by, bz]
  private state: number[];
  
  // State covariance matrix (10x10)
  private P: number[][];
  
  // Process noise covariance
  private Q: number[][];
  
  // Measurement noise for accelerometer (3x3)
  private R_accel: number[][];
  
  // Measurement noise for magnetometer (3x3)
  private R_mag: number[][];
  
  // Gravity vector in inertial frame (NED convention)  
  // Standard roll/pitch formulas assume gravity points DOWN (+Z in NED)
  // Even though accelerometer measures upward reaction force, we represent gravity as downward
  // The formulas account for this by using the measured acceleration directly
  private readonly g: { x: number; y: number; z: number } = { x: 0, y: 0, z: 9.81 };
  
  // Earth's magnetic field vector in inertial frame (pointing north)
  // Magnitude doesn't matter since we normalize - only direction matters
  private mag_earth: { x: number; y: number; z: number } = { x: 1.0, y: 0, z: 0 };
  
  private lastUpdateTime: number = 0;
  private initialized: boolean = false;

  constructor(
    gyroNoiseVar: number = 0.001,      // Gyroscope noise variance (rad/s)²
    gyroBiasNoiseVar: number = 0.0001, // Gyro bias random walk (rad/s)²
    accelNoiseVar: number = 0.1,       // Accelerometer noise variance (m/s²)²
    magNoiseVar: number = 0.1          // Magnetometer noise variance (normalized)²
  ) {
    // Initialize state to identity orientation, zero velocity, zero bias
    this.state = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    
    // Initialize covariance matrix with high initial uncertainty
    this.P = this.createIdentityMatrix(10);
    for (let i = 0; i < 4; i++) {
      this.P[i][i] = 0.1;  // Quaternion uncertainty
    }
    for (let i = 4; i < 7; i++) {
      this.P[i][i] = 0.01; // Angular velocity uncertainty
    }
    for (let i = 7; i < 10; i++) {
      this.P[i][i] = 0.001; // Gyro bias uncertainty
    }
    
    // Process noise covariance (continuous time, will be discretized)
    this.Q = this.createZeroMatrix(10);
    // Quaternion process noise (from gyro integration)
    for (let i = 0; i < 4; i++) {
      this.Q[i][i] = gyroNoiseVar * 0.01;
    }
    // Angular velocity process noise
    for (let i = 4; i < 7; i++) {
      this.Q[i][i] = gyroNoiseVar;
    }
    // Gyro bias random walk
    for (let i = 7; i < 10; i++) {
      this.Q[i][i] = gyroBiasNoiseVar;
    }
    
    // Accelerometer measurement noise
    this.R_accel = [
      [accelNoiseVar, 0, 0],
      [0, accelNoiseVar, 0],
      [0, 0, accelNoiseVar]
    ];
    
    // Magnetometer measurement noise
    this.R_mag = [
      [magNoiseVar, 0, 0],
      [0, magNoiseVar, 0],
      [0, 0, magNoiseVar]
    ];
  }

  /**
   * Initialize filter with initial orientation from accelerometer and magnetometer
   * Should only be called when device is stationary!
   * 
   * @param gyro - Optional gyroscope reading to verify device is stationary
   */
  initialize(accel: { x: number; y: number; z: number }, 
             mag: { x: number; y: number; z: number },
             timestamp: number,
             gyro?: { x: number; y: number; z: number }): void {
    // Check if device is approximately stationary (accel magnitude ≈ gravity)
    const a_norm = Math.sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    const gravity_mag = 9.81;
    if (Math.abs(a_norm - gravity_mag) > 1.0) {
      console.warn(`Cannot initialize: device is moving! Accel magnitude: ${a_norm.toFixed(2)} m/s² (expected ~9.81)`);
      return;
    }
    
    // Check gyroscope if provided (should be near zero when stationary)
    if (gyro) {
      const gyro_norm = Math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2);
      const gyro_deg = gyro_norm * 180 / Math.PI;
      // Use relaxed threshold (20 deg/s) to allow initialization in less-than-perfect conditions
      // During skydiving, finding perfectly still moments is rare
      if (gyro_deg > 20.0) {
        console.warn(`Cannot initialize: device is rotating! Gyro magnitude: ${gyro_deg.toFixed(2)} deg/s (should be <20)`);
        return;
      }
    }
    
    if (a_norm < 0.1) return;
    
    // Accelerometer measures reaction force (opposite of gravity)
    // Negate to get gravity direction for standard roll/pitch formulas
    const ax = -accel.x / a_norm;
    const ay = -accel.y / a_norm;
    const az = -accel.z / a_norm;
    
    // Calculate roll and pitch from accelerometer
    // Standard formulas: roll = atan2(ay, az), pitch = atan2(-ax, sqrt(ay²+az²))
    // where (ax, ay, az) represents gravity direction (down)
    const roll = Math.atan2(ay, az);
    const pitch = Math.atan2(-ax, Math.sqrt(ay * ay + az * az));
    
    // Normalize magnetometer
    const m_norm = Math.sqrt(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
    if (m_norm < 0.001) {
      // Initialize without magnetometer (roll and pitch only)
      const q = Quaternion.fromEuler(0, pitch, roll);
      this.state[0] = q.w;
      this.state[1] = q.x;
      this.state[2] = q.y;
      this.state[3] = q.z;
    } else {
      const mx = mag.x / m_norm;
      const my = mag.y / m_norm;
      const mz = mag.z / m_norm;
      
      // Tilt-compensated heading
      const mx_h = mx * Math.cos(pitch) + mz * Math.sin(pitch);
      const my_h = mx * Math.sin(roll) * Math.sin(pitch) + my * Math.cos(roll) - mz * Math.sin(roll) * Math.cos(pitch);
      const yaw = Math.atan2(-my_h, mx_h);
      
      // Initialize quaternion
      const q = Quaternion.fromEuler(yaw, pitch, roll);
      this.state[0] = q.w;
      this.state[1] = q.x;
      this.state[2] = q.y;
      this.state[3] = q.z;
      
      // Set mag_earth by rotating measured mag from body to NED frame
      // This captures the actual local magnetic field direction (including dip angle)
      this.mag_earth = q.rotateVector({ x: mx, y: my, z: mz });
      console.log('Initialized mag_earth (NED frame):', this.mag_earth);
    }
    
    // Reset covariance to reflect confidence in initial orientation from accel/mag
    // Quaternion uncertainty should be small since we just computed it
    for (let i = 0; i < 4; i++) {
      this.P[i][i] = 0.001;  // High confidence in initial quaternion
    }
    
    this.lastUpdateTime = timestamp;
    this.initialized = true;
  }

  /**
   * Predict step: propagate state using gyroscope measurements
   */
  predict(gyro: { x: number; y: number; z: number }, timestamp: number): void {
    if (!this.initialized) return;
    
    const dt = timestamp - this.lastUpdateTime;
    if (dt <= 0 || dt > 1.0) {
      this.lastUpdateTime = timestamp;
      return;
    }
    
    // Extract current state
    const q = new Quaternion(this.state[0], this.state[1], this.state[2], this.state[3]);
    const bias = { x: this.state[7], y: this.state[8], z: this.state[9] };
    
    // Bias-corrected angular velocity
    const omega = {
      x: gyro.x - bias.x,
      y: gyro.y - bias.y,
      z: gyro.z - bias.z
    };
    
    // Update angular velocity in state
    this.state[4] = omega.x;
    this.state[5] = omega.y;
    this.state[6] = omega.z;
    
    // Integrate quaternion using RK4 (4th order Runge-Kutta)
    // This is much more accurate than Euler integration for low sample rates
    const q_new = this.integrateQuaternionRK4(q, omega, dt);
    
    // Update state
    this.state[0] = q_new.w;
    this.state[1] = q_new.x;
    this.state[2] = q_new.y;
    this.state[3] = q_new.z;
    
    // State transition Jacobian (F)
    const F = this.computeStateTransitionJacobian(omega, dt);
    
    // Covariance prediction: P = F * P * F^T + Q * dt
    const FP = this.multiplyMatrices(F, this.P);
    const FP_FT = this.multiplyMatrices(FP, this.transposeMatrix(F));
    
    // Discretize Q by multiplying by dt
    const Q_discrete = this.scaleMatrix(this.Q, dt);
    
    this.P = this.addMatrices(FP_FT, Q_discrete);
    
    this.lastUpdateTime = timestamp;
  }

  /**
   * Update step: correct state using accelerometer measurement
   */
  updateAccelerometer(accel: { x: number; y: number; z: number }): void {
    if (!this.initialized) return;
    
    // Check measurement validity
    const a_norm = Math.sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    if (a_norm < 0.1) return; // Invalid measurement
    
    // Motion detection: check if magnitude deviates from gravity (9.81 m/s²)
    // If there's linear acceleration, reduce trust in accelerometer
    const gravity_mag = 9.81;
    const accel_deviation = Math.abs(a_norm - gravity_mag);
    const motion_threshold = 2.0; // 2 m/s² deviation indicates motion
    
    // Adaptive noise: increase R_accel during motion
    // Scale factor: 1.0 when static, up to 100× when moving fast
    const motion_scale = 1.0 + Math.min(99.0, (accel_deviation / motion_threshold) ** 2);
    
    const R_accel_adaptive = [
      [this.R_accel[0][0] * motion_scale, 0, 0],
      [0, this.R_accel[1][1] * motion_scale, 0],
      [0, 0, this.R_accel[2][2] * motion_scale]
    ];
    
    // Debug logging
    if (Math.random() < 0.01) {
      console.log('ACCEL update:', {
        a_norm: a_norm.toFixed(2),
        deviation: accel_deviation.toFixed(2),
        motion_scale: motion_scale.toFixed(2),
        R_base: this.R_accel[0][0].toFixed(4),
        R_adaptive: R_accel_adaptive[0][0].toFixed(4)
      });
    }
    
    // Normalize measured acceleration to unit vector
    // Accelerometer measures reaction force (opposite of gravity)
    const a_measured = { 
      x: accel.x / a_norm, 
      y: accel.y / a_norm, 
      z: accel.z / a_norm 
    };
    
    // Expected measurement: rotate normalized gravity DOWN vector to body frame
    // Then negate to match accelerometer (which measures UP)
    const q = new Quaternion(this.state[0], this.state[1], this.state[2], this.state[3]);
    const g_norm = Math.sqrt(this.g.x**2 + this.g.y**2 + this.g.z**2);
    const g_normalized = { x: this.g.x / g_norm, y: this.g.y / g_norm, z: this.g.z / g_norm };
    const g_rotated = q.conjugate().rotateVector(g_normalized);
    // Negate because accelerometer measures opposite of gravity
    const a_expected = { x: -g_rotated.x, y: -g_rotated.y, z: -g_rotated.z };
    
    // Innovation: difference between measurement and prediction (unitless)
    const y = [
      a_measured.x - a_expected.x,
      a_measured.y - a_expected.y,
      a_measured.z - a_expected.z
    ];
    
    // Debug logging
    if (Math.random() < 0.01) {
      const innovation_norm = Math.sqrt(y[0]**2 + y[1]**2 + y[2]**2);
      console.log('ACCEL innovation:', {
        innovation: innovation_norm.toFixed(4),
        measured: {x: a_measured.x.toFixed(3), y: a_measured.y.toFixed(3), z: a_measured.z.toFixed(3)},
        expected: {x: a_expected.x.toFixed(3), y: a_expected.y.toFixed(3), z: a_expected.z.toFixed(3)}
      });
    }
    
    // Measurement Jacobian H (3x10): ∂h/∂x
    const H = this.computeAccelMeasurementJacobian(q);
    
    // Innovation covariance: S = H * P * H^T + R_adaptive
    const HP = this.multiplyMatrices(H, this.P);
    const HP_HT = this.multiplyMatrices(HP, this.transposeMatrix(H));
    const S = this.addMatrices(HP_HT, R_accel_adaptive);
    
    // Kalman gain: K = P * H^T * S^(-1)
    const S_inv = this.invertMatrix3x3(S);
    const P_HT = this.multiplyMatrices(this.P, this.transposeMatrix(H));
    const K = this.multiplyMatrices(P_HT, S_inv);
    
    // State update: x = x + K * y
    for (let i = 0; i < 10; i++) {
      for (let j = 0; j < 3; j++) {
        this.state[i] += K[i][j] * y[j];
      }
    }
    
    // Normalize quaternion
    const q_updated = new Quaternion(this.state[0], this.state[1], this.state[2], this.state[3]).normalize();
    this.state[0] = q_updated.w;
    this.state[1] = q_updated.x;
    this.state[2] = q_updated.y;
    this.state[3] = q_updated.z;
    
    // Clamp gyro bias to reasonable bounds (prevent divergence)
    const maxBias = 0.5; // 0.5 rad/s = ~28.6 deg/s max bias
    for (let i = 7; i < 10; i++) {
      this.state[i] = Math.max(-maxBias, Math.min(maxBias, this.state[i]));
    }
    
    // Covariance update: P = (I - K * H) * P
    const I = this.createIdentityMatrix(10);
    const KH = this.multiplyMatrices(K, H);
    const I_KH = this.subtractMatrices(I, KH);
    this.P = this.multiplyMatrices(I_KH, this.P);
  }

  /**
   * Update step: correct state using magnetometer measurement
   */
  updateMagnetometer(mag: { x: number; y: number; z: number }): void {
    if (!this.initialized) return;
    
    // Check measurement validity
    const m_norm = Math.sqrt(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
    if (m_norm < 0.001) return; // Invalid measurement
    
    // Normalize measured magnetic field to unit vector
    const m_measured = { 
      x: mag.x / m_norm, 
      y: mag.y / m_norm, 
      z: mag.z / m_norm 
    };
    
    // Expected measurement: rotate normalized earth field to body frame
    const q = new Quaternion(this.state[0], this.state[1], this.state[2], this.state[3]);
    const mag_norm = Math.sqrt(this.mag_earth.x**2 + this.mag_earth.y**2 + this.mag_earth.z**2);
    const mag_normalized = { 
      x: this.mag_earth.x / mag_norm, 
      y: this.mag_earth.y / mag_norm, 
      z: this.mag_earth.z / mag_norm 
    };
    const m_expected = q.conjugate().rotateVector(mag_normalized);
    
    // Innovation (gauss)
    const y = [
      m_measured.x - m_expected.x,
      m_measured.y - m_expected.y,
      m_measured.z - m_expected.z
    ];
    
    // Debug: log innovation and R_mag to verify noise parameter effect
    const innovation_norm = Math.sqrt(y[0]**2 + y[1]**2 + y[2]**2);
    if (Math.random() < 0.01) { // Log 1% of updates to avoid spam
      console.log('MAG update:', {
        innovation: innovation_norm.toFixed(4),
        R_mag_diagonal: this.R_mag[0][0].toFixed(4),
        measured: m_measured,
        expected: m_expected
      });
    }
    
    // Measurement Jacobian H (3x10): ∂h/∂x
    const H = this.computeMagMeasurementJacobian(q);
    
    // Innovation covariance: S = H * P * H^T + R
    const HP = this.multiplyMatrices(H, this.P);
    const HP_HT = this.multiplyMatrices(HP, this.transposeMatrix(H));
    const S = this.addMatrices(HP_HT, this.R_mag);
    
    // Kalman gain: K = P * H^T * S^(-1)
    const S_inv = this.invertMatrix3x3(S);
    const P_HT = this.multiplyMatrices(this.P, this.transposeMatrix(H));
    const K = this.multiplyMatrices(P_HT, S_inv);
    
    // State update: x = x + K * y
    for (let i = 0; i < 10; i++) {
      for (let j = 0; j < 3; j++) {
        this.state[i] += K[i][j] * y[j];
      }
    }
    
    // Normalize quaternion
    const q_updated = new Quaternion(this.state[0], this.state[1], this.state[2], this.state[3]).normalize();
    this.state[0] = q_updated.w;
    this.state[1] = q_updated.x;
    this.state[2] = q_updated.y;
    this.state[3] = q_updated.z;
    
    // Clamp gyro bias to reasonable bounds (prevent divergence)
    const maxBias = 0.5; // 0.5 rad/s = ~28.6 deg/s max bias
    for (let i = 7; i < 10; i++) {
      this.state[i] = Math.max(-maxBias, Math.min(maxBias, this.state[i]));
    }
    
    // Covariance update: P = (I - K * H) * P
    const I = this.createIdentityMatrix(10);
    const KH = this.multiplyMatrices(K, H);
    const I_KH = this.subtractMatrices(I, KH);
    this.P = this.multiplyMatrices(I_KH, this.P);
  }

  /**
   * Get current orientation as quaternion
   */
  getQuaternion(): Quaternion {
    return new Quaternion(this.state[0], this.state[1], this.state[2], this.state[3]);
  }

  /**
   * Get current orientation as Euler angles (radians)
   */
  getEulerAngles(): { yaw: number; pitch: number; roll: number } {
    return this.getQuaternion().toEuler();
  }

  /**
   * Get current angular velocity (rad/s)
   */
  getAngularVelocity(): { x: number; y: number; z: number } {
    return { x: this.state[4], y: this.state[5], z: this.state[6] };
  }

  /**
   * Get current gyroscope bias estimate (rad/s)
   */
  getGyroBias(): { x: number; y: number; z: number } {
    return { x: this.state[7], y: this.state[8], z: this.state[9] };
  }

  /**
   * Set earth's magnetic field vector (unnormalized, in inertial frame, gauss)
   */
  setMagneticField(mag: { x: number; y: number; z: number }): void {
    this.mag_earth = { x: mag.x, y: mag.y, z: mag.z };
  }

  isInitialized(): boolean {
    return this.initialized;
  }

  /**
   * Integrate quaternion using 4th-order Runge-Kutta method
   * More accurate than Euler integration, especially at low sample rates
   * 
   * @param q Current quaternion
   * @param omega Angular velocity (rad/s)
   * @param dt Time step (seconds)
   * @returns Integrated quaternion (normalized)
   */
  private integrateQuaternionRK4(
    q: Quaternion, 
    omega: { x: number; y: number; z: number }, 
    dt: number
  ): Quaternion {
    // Helper function to compute quaternion derivative: dq/dt = 0.5 * q * ω
    const quaternionDerivative = (q_in: Quaternion, w: { x: number; y: number; z: number }) => {
      const omega_q = new Quaternion(0, w.x, w.y, w.z);
      const q_dot = q_in.multiply(omega_q);
      return new Quaternion(
        0.5 * q_dot.w,
        0.5 * q_dot.x,
        0.5 * q_dot.y,
        0.5 * q_dot.z
      );
    };

    // RK4 integration: k1, k2, k3, k4 are the derivatives at different points
    // k1 = f(t, q)
    const k1 = quaternionDerivative(q, omega);

    // k2 = f(t + dt/2, q + dt/2 * k1)
    const q2 = new Quaternion(
      q.w + 0.5 * dt * k1.w,
      q.x + 0.5 * dt * k1.x,
      q.y + 0.5 * dt * k1.y,
      q.z + 0.5 * dt * k1.z
    );
    const k2 = quaternionDerivative(q2, omega);

    // k3 = f(t + dt/2, q + dt/2 * k2)
    const q3 = new Quaternion(
      q.w + 0.5 * dt * k2.w,
      q.x + 0.5 * dt * k2.x,
      q.y + 0.5 * dt * k2.y,
      q.z + 0.5 * dt * k2.z
    );
    const k3 = quaternionDerivative(q3, omega);

    // k4 = f(t + dt, q + dt * k3)
    const q4 = new Quaternion(
      q.w + dt * k3.w,
      q.x + dt * k3.x,
      q.y + dt * k3.y,
      q.z + dt * k3.z
    );
    const k4 = quaternionDerivative(q4, omega);

    // Combine: q_new = q + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    const q_new = new Quaternion(
      q.w + (dt / 6.0) * (k1.w + 2 * k2.w + 2 * k3.w + k4.w),
      q.x + (dt / 6.0) * (k1.x + 2 * k2.x + 2 * k3.x + k4.x),
      q.y + (dt / 6.0) * (k1.y + 2 * k2.y + 2 * k3.y + k4.y),
      q.z + (dt / 6.0) * (k1.z + 2 * k2.z + 2 * k3.z + k4.z)
    );

    // Normalize to maintain unit quaternion constraint
    return q_new.normalize();
  }

  /**
   * Compute state transition Jacobian F (10x10)
   * Linearization of the state dynamics
   */
  private computeStateTransitionJacobian(omega: { x: number; y: number; z: number }, dt: number): number[][] {
    const F = this.createIdentityMatrix(10);
    
    const wx = omega.x, wy = omega.y, wz = omega.z;
    const q0 = this.state[0], q1 = this.state[1], q2 = this.state[2], q3 = this.state[3];
    
    // dq/dq (quaternion evolution from quaternion)
    const Omega = [
      [0, -wx, -wy, -wz],
      [wx, 0, wz, -wy],
      [wy, -wz, 0, wx],
      [wz, wy, -wx, 0]
    ];
    
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        F[i][j] += 0.5 * Omega[i][j] * dt;
      }
    }
    
    // dq/dbias (quaternion evolution from bias)
    F[0][7] = 0.5 * q1 * dt;
    F[0][8] = 0.5 * q2 * dt;
    F[0][9] = 0.5 * q3 * dt;
    
    F[1][7] = -0.5 * q0 * dt;
    F[1][8] = -0.5 * q3 * dt;
    F[1][9] = 0.5 * q2 * dt;
    
    F[2][7] = 0.5 * q3 * dt;
    F[2][8] = -0.5 * q0 * dt;
    F[2][9] = -0.5 * q1 * dt;
    
    F[3][7] = -0.5 * q2 * dt;
    F[3][8] = 0.5 * q1 * dt;
    F[3][9] = -0.5 * q0 * dt;
    
    return F;
  }

  /**
   * Compute accelerometer measurement Jacobian H (3x10)
   * ∂(-R^T * g)/∂x where R is rotation from q
   * Note: Accelerometer measures -g (reaction force), so we negate the Jacobian
   */
  private computeAccelMeasurementJacobian(q: Quaternion): number[][] {
    const H = this.createZeroMatrix(3, 10);
    
    const q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
    // Use normalized gravity for Jacobian
    const g_norm = Math.sqrt(this.g.x**2 + this.g.y**2 + this.g.z**2);
    const gx = this.g.x / g_norm, gy = this.g.y / g_norm, gz = this.g.z / g_norm;
    
    // Jacobian of rotation with respect to quaternion
    // h = -R^T(q) * g (negated because accelerometer measures opposite of gravity)
    
    H[0][0] = -2 * (q0*gx - q3*gy + q2*gz);
    H[0][1] = -2 * (q1*gx + q2*gy + q3*gz);
    H[0][2] = -2 * (-q2*gx + q1*gy - q0*gz);
    H[0][3] = -2 * (-q3*gx + q0*gy + q1*gz);
    
    H[1][0] = -2 * (q3*gx + q0*gy - q1*gz);
    H[1][1] = -2 * (q2*gx - q1*gy - q0*gz);
    H[1][2] = -2 * (q1*gx + q2*gy + q3*gz);
    H[1][3] = -2 * (q0*gx - q3*gy + q2*gz);
    
    H[2][0] = -2 * (-q2*gx + q1*gy - q0*gz);
    H[2][1] = -2 * (q3*gx + q0*gy - q1*gz);
    H[2][2] = -2 * (-q0*gx + q3*gy - q2*gz);
    H[2][3] = -2 * (q1*gx + q2*gy + q3*gz);
    
    // No dependence on angular velocity or bias
    return H;
  }

  /**
   * Compute magnetometer measurement Jacobian H (3x10)
   */
  private computeMagMeasurementJacobian(q: Quaternion): number[][] {
    const H = this.createZeroMatrix(3, 10);
    
    const q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
    // Use normalized magnetic field for Jacobian
    const mag_norm = Math.sqrt(this.mag_earth.x**2 + this.mag_earth.y**2 + this.mag_earth.z**2);
    const mx = this.mag_earth.x / mag_norm, my = this.mag_earth.y / mag_norm, mz = this.mag_earth.z / mag_norm;
    
    // Similar to accelerometer Jacobian but with magnetic field
    H[0][0] = 2 * (q0*mx - q3*my + q2*mz);
    H[0][1] = 2 * (q1*mx + q2*my + q3*mz);
    H[0][2] = 2 * (-q2*mx + q1*my - q0*mz);
    H[0][3] = 2 * (-q3*mx + q0*my + q1*mz);
    
    H[1][0] = 2 * (q3*mx + q0*my - q1*mz);
    H[1][1] = 2 * (q2*mx - q1*my - q0*mz);
    H[1][2] = 2 * (q1*mx + q2*my + q3*mz);
    H[1][3] = 2 * (q0*mx - q3*my + q2*mz);
    
    H[2][0] = 2 * (-q2*mx + q1*my - q0*mz);
    H[2][1] = 2 * (q3*mx + q0*my - q1*mz);
    H[2][2] = 2 * (-q0*mx + q3*my - q2*mz);
    H[2][3] = 2 * (q1*mx + q2*my + q3*mz);
    
    return H;
  }

  // Matrix utility functions
  
  private createZeroMatrix(rows: number, cols?: number): number[][] {
    cols = cols || rows;
    const matrix: number[][] = [];
    for (let i = 0; i < rows; i++) {
      matrix[i] = new Array(cols).fill(0);
    }
    return matrix;
  }

  private createIdentityMatrix(n: number): number[][] {
    const matrix = this.createZeroMatrix(n);
    for (let i = 0; i < n; i++) {
      matrix[i][i] = 1;
    }
    return matrix;
  }

  private multiplyMatrices(A: number[][], B: number[][]): number[][] {
    const rows = A.length;
    const cols = B[0].length;
    const inner = B.length;
    const result = this.createZeroMatrix(rows, cols);
    
    for (let i = 0; i < rows; i++) {
      for (let j = 0; j < cols; j++) {
        for (let k = 0; k < inner; k++) {
          result[i][j] += A[i][k] * B[k][j];
        }
      }
    }
    return result;
  }

  private addMatrices(A: number[][], B: number[][]): number[][] {
    const rows = A.length;
    const cols = A[0].length;
    const result = this.createZeroMatrix(rows, cols);
    
    for (let i = 0; i < rows; i++) {
      for (let j = 0; j < cols; j++) {
        result[i][j] = A[i][j] + B[i][j];
      }
    }
    return result;
  }

  private subtractMatrices(A: number[][], B: number[][]): number[][] {
    const rows = A.length;
    const cols = A[0].length;
    const result = this.createZeroMatrix(rows, cols);
    
    for (let i = 0; i < rows; i++) {
      for (let j = 0; j < cols; j++) {
        result[i][j] = A[i][j] - B[i][j];
      }
    }
    return result;
  }

  private transposeMatrix(A: number[][]): number[][] {
    const rows = A.length;
    const cols = A[0].length;
    const result = this.createZeroMatrix(cols, rows);
    
    for (let i = 0; i < rows; i++) {
      for (let j = 0; j < cols; j++) {
        result[j][i] = A[i][j];
      }
    }
    return result;
  }

  private scaleMatrix(A: number[][], scalar: number): number[][] {
    const rows = A.length;
    const cols = A[0].length;
    const result = this.createZeroMatrix(rows, cols);
    
    for (let i = 0; i < rows; i++) {
      for (let j = 0; j < cols; j++) {
        result[i][j] = A[i][j] * scalar;
      }
    }
    return result;
  }

  private invertMatrix3x3(A: number[][]): number[][] {
    // Compute determinant
    const det = 
      A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
      A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
      A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    
    if (Math.abs(det) < 1e-10) {
      // Singular matrix, return identity
      return this.createIdentityMatrix(3);
    }
    
    const invDet = 1.0 / det;
    const result = this.createZeroMatrix(3);
    
    result[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invDet;
    result[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * invDet;
    result[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet;
    
    result[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * invDet;
    result[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet;
    result[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * invDet;
    
    result[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invDet;
    result[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * invDet;
    result[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invDet;
    
    return result;
  }
}
