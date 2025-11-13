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
  
  // Gravity vector in inertial frame (NED convention: down is positive)
  private readonly g: { x: number; y: number; z: number } = { x: 0, y: 0, z: 9.81 };
  
  // Earth's magnetic field vector in inertial frame (normalized, pointing north)
  // This should be set based on location; default is simplified horizontal north
  private mag_earth: { x: number; y: number; z: number } = { x: 1, y: 0, z: 0 };
  
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
   */
  initialize(accel: { x: number; y: number; z: number }, 
             mag: { x: number; y: number; z: number },
             timestamp: number): void {
    // Normalize accelerometer
    const a_norm = Math.sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    if (a_norm < 0.1) return;
    
    const ax = accel.x / a_norm;
    const ay = accel.y / a_norm;
    const az = accel.z / a_norm;
    
    // Calculate roll and pitch from accelerometer
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
    
    // Quaternion derivative: dq/dt = 0.5 * q * ω
    const omega_q = new Quaternion(0, omega.x, omega.y, omega.z);
    const q_dot = q.multiply(omega_q);
    
    // Integrate quaternion (Euler integration)
    const q_new = new Quaternion(
      q.w + 0.5 * q_dot.w * dt,
      q.x + 0.5 * q_dot.x * dt,
      q.y + 0.5 * q_dot.y * dt,
      q.z + 0.5 * q_dot.z * dt
    ).normalize();
    
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
    
    // Normalize accelerometer measurement
    const a_norm = Math.sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    if (a_norm < 0.1) return; // Invalid measurement
    
    const a_measured = {
      x: accel.x / a_norm,
      y: accel.y / a_norm,
      z: accel.z / a_norm
    };
    
    // Expected measurement: rotate gravity vector to body frame
    const q = new Quaternion(this.state[0], this.state[1], this.state[2], this.state[3]);
    const a_expected = q.conjugate().rotateVector(this.g);
    
    // Normalize expected (should already be normalized, but ensure)
    const a_exp_norm = Math.sqrt(a_expected.x * a_expected.x + a_expected.y * a_expected.y + a_expected.z * a_expected.z);
    a_expected.x /= a_exp_norm;
    a_expected.y /= a_exp_norm;
    a_expected.z /= a_exp_norm;
    
    // Innovation: difference between measurement and prediction
    const y = [
      a_measured.x - a_expected.x,
      a_measured.y - a_expected.y,
      a_measured.z - a_expected.z
    ];
    
    // Measurement Jacobian H (3x10): ∂h/∂x
    const H = this.computeAccelMeasurementJacobian(q);
    
    // Innovation covariance: S = H * P * H^T + R
    const HP = this.multiplyMatrices(H, this.P);
    const HP_HT = this.multiplyMatrices(HP, this.transposeMatrix(H));
    const S = this.addMatrices(HP_HT, this.R_accel);
    
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
    
    // Normalize magnetometer measurement
    const m_norm = Math.sqrt(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
    if (m_norm < 0.001) return; // Invalid measurement
    
    const m_measured = {
      x: mag.x / m_norm,
      y: mag.y / m_norm,
      z: mag.z / m_norm
    };
    
    // Expected measurement: rotate earth's magnetic field to body frame
    const q = new Quaternion(this.state[0], this.state[1], this.state[2], this.state[3]);
    const m_expected = q.conjugate().rotateVector(this.mag_earth);
    
    // Normalize expected
    const m_exp_norm = Math.sqrt(m_expected.x * m_expected.x + m_expected.y * m_expected.y + m_expected.z * m_expected.z);
    m_expected.x /= m_exp_norm;
    m_expected.y /= m_exp_norm;
    m_expected.z /= m_exp_norm;
    
    // Innovation
    const y = [
      m_measured.x - m_expected.x,
      m_measured.y - m_expected.y,
      m_measured.z - m_expected.z
    ];
    
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
   * Set earth's magnetic field vector (normalized, in inertial frame)
   */
  setMagneticField(mag: { x: number; y: number; z: number }): void {
    const norm = Math.sqrt(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
    this.mag_earth = {
      x: mag.x / norm,
      y: mag.y / norm,
      z: mag.z / norm
    };
  }

  isInitialized(): boolean {
    return this.initialized;
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
   * ∂(R^T * g)/∂x where R is rotation from q
   */
  private computeAccelMeasurementJacobian(q: Quaternion): number[][] {
    const H = this.createZeroMatrix(3, 10);
    
    const q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
    const gx = this.g.x, gy = this.g.y, gz = this.g.z;
    
    // Jacobian of rotation with respect to quaternion
    // h = R^T(q) * g, where R^T is the conjugate rotation
    
    H[0][0] = 2 * (q0*gx - q3*gy + q2*gz);
    H[0][1] = 2 * (q1*gx + q2*gy + q3*gz);
    H[0][2] = 2 * (-q2*gx + q1*gy - q0*gz);
    H[0][3] = 2 * (-q3*gx + q0*gy + q1*gz);
    
    H[1][0] = 2 * (q3*gx + q0*gy - q1*gz);
    H[1][1] = 2 * (q2*gx - q1*gy - q0*gz);
    H[1][2] = 2 * (q1*gx + q2*gy + q3*gz);
    H[1][3] = 2 * (q0*gx - q3*gy + q2*gz);
    
    H[2][0] = 2 * (-q2*gx + q1*gy - q0*gz);
    H[2][1] = 2 * (q3*gx + q0*gy - q1*gz);
    H[2][2] = 2 * (-q0*gx + q3*gy - q2*gz);
    H[2][3] = 2 * (q1*gx + q2*gy + q3*gz);
    
    // No dependence on angular velocity or bias
    return H;
  }

  /**
   * Compute magnetometer measurement Jacobian H (3x10)
   */
  private computeMagMeasurementJacobian(q: Quaternion): number[][] {
    const H = this.createZeroMatrix(3, 10);
    
    const q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
    const mx = this.mag_earth.x, my = this.mag_earth.y, mz = this.mag_earth.z;
    
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
