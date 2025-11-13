/**
 * Quaternion class for 3D rotations
 * Quaternion format: q = w + xi + yj + zk
 * where w is the scalar part and (x, y, z) is the vector part
 */
export class Quaternion {
  w: number;  // Scalar part
  x: number;  // Vector part i
  y: number;  // Vector part j
  z: number;  // Vector part k

  constructor(w: number = 1, x: number = 0, y: number = 0, z: number = 0) {
    this.w = w;
    this.x = x;
    this.y = y;
    this.z = z;
  }

  /**
   * Create identity quaternion (no rotation)
   */
  static identity(): Quaternion {
    return new Quaternion(1, 0, 0, 0);
  }

  /**
   * Create quaternion from Euler angles (ZYX order)
   * @param yaw Rotation around Z axis (radians)
   * @param pitch Rotation around Y axis (radians)
   * @param roll Rotation around X axis (radians)
   */
  static fromEuler(yaw: number, pitch: number, roll: number): Quaternion {
    const cy = Math.cos(yaw * 0.5);
    const sy = Math.sin(yaw * 0.5);
    const cp = Math.cos(pitch * 0.5);
    const sp = Math.sin(pitch * 0.5);
    const cr = Math.cos(roll * 0.5);
    const sr = Math.sin(roll * 0.5);

    return new Quaternion(
      cr * cp * cy + sr * sp * sy,  // w
      sr * cp * cy - cr * sp * sy,  // x
      cr * sp * cy + sr * cp * sy,  // y
      cr * cp * sy - sr * sp * cy   // z
    );
  }

  /**
   * Create quaternion from axis-angle representation
   * @param axis Unit vector representing rotation axis
   * @param angle Rotation angle in radians
   */
  static fromAxisAngle(axis: { x: number; y: number; z: number }, angle: number): Quaternion {
    const halfAngle = angle * 0.5;
    const s = Math.sin(halfAngle);
    return new Quaternion(
      Math.cos(halfAngle),
      axis.x * s,
      axis.y * s,
      axis.z * s
    );
  }

  /**
   * Convert quaternion to Euler angles (ZYX order)
   * @returns {yaw, pitch, roll} in radians
   */
  toEuler(): { yaw: number; pitch: number; roll: number } {
    // Roll (x-axis rotation)
    const sinr_cosp = 2 * (this.w * this.x + this.y * this.z);
    const cosr_cosp = 1 - 2 * (this.x * this.x + this.y * this.y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    const sinp = 2 * (this.w * this.y - this.z * this.x);
    const pitch = Math.abs(sinp) >= 1
      ? Math.sign(sinp) * Math.PI / 2  // Use 90 degrees if out of range
      : Math.asin(sinp);

    // Yaw (z-axis rotation)
    const siny_cosp = 2 * (this.w * this.z + this.x * this.y);
    const cosy_cosp = 1 - 2 * (this.y * this.y + this.z * this.z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);

    return { yaw, pitch, roll };
  }

  /**
   * Quaternion multiplication (Hamilton product)
   * Represents composition of rotations: this * other
   */
  multiply(other: Quaternion): Quaternion {
    return new Quaternion(
      this.w * other.w - this.x * other.x - this.y * other.y - this.z * other.z,
      this.w * other.x + this.x * other.w + this.y * other.z - this.z * other.y,
      this.w * other.y - this.x * other.z + this.y * other.w + this.z * other.x,
      this.w * other.z + this.x * other.y - this.y * other.x + this.z * other.w
    );
  }

  /**
   * Quaternion conjugate (inverse for unit quaternions)
   */
  conjugate(): Quaternion {
    return new Quaternion(this.w, -this.x, -this.y, -this.z);
  }

  /**
   * Quaternion norm (magnitude)
   */
  norm(): number {
    return Math.sqrt(this.w * this.w + this.x * this.x + this.y * this.y + this.z * this.z);
  }

  /**
   * Normalize quaternion to unit length
   */
  normalize(): Quaternion {
    const n = this.norm();
    if (n < 1e-10) {
      return Quaternion.identity();
    }
    return new Quaternion(this.w / n, this.x / n, this.y / n, this.z / n);
  }

  /**
   * Rotate a 3D vector by this quaternion
   * v' = q * v * q^*
   */
  rotateVector(v: { x: number; y: number; z: number }): { x: number; y: number; z: number } {
    // Convert vector to quaternion
    const vq = new Quaternion(0, v.x, v.y, v.z);
    
    // Perform rotation: q * v * q^*
    const result = this.multiply(vq).multiply(this.conjugate());
    
    return { x: result.x, y: result.y, z: result.z };
  }

  /**
   * Convert quaternion to 3x3 rotation matrix
   */
  toRotationMatrix(): number[][] {
    const w = this.w, x = this.x, y = this.y, z = this.z;
    
    return [
      [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
      [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
      [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ];
  }

  /**
   * Linear interpolation between two quaternions
   * @param other Target quaternion
   * @param t Interpolation parameter [0, 1]
   */
  lerp(other: Quaternion, t: number): Quaternion {
    return new Quaternion(
      this.w + t * (other.w - this.w),
      this.x + t * (other.x - this.x),
      this.y + t * (other.y - this.y),
      this.z + t * (other.z - this.z)
    ).normalize();
  }

  /**
   * Spherical linear interpolation between two quaternions
   * @param other Target quaternion
   * @param t Interpolation parameter [0, 1]
   */
  slerp(other: Quaternion, t: number): Quaternion {
    // Compute dot product
    let dot = this.w * other.w + this.x * other.x + this.y * other.y + this.z * other.z;
    
    // If negative, use -other to take shorter path
    let q2 = other;
    if (dot < 0) {
      q2 = new Quaternion(-other.w, -other.x, -other.y, -other.z);
      dot = -dot;
    }
    
    // If quaternions are very close, use linear interpolation
    if (dot > 0.9995) {
      return this.lerp(q2, t);
    }
    
    // Perform spherical linear interpolation
    const theta = Math.acos(dot);
    const sinTheta = Math.sin(theta);
    const w1 = Math.sin((1 - t) * theta) / sinTheta;
    const w2 = Math.sin(t * theta) / sinTheta;
    
    return new Quaternion(
      w1 * this.w + w2 * q2.w,
      w1 * this.x + w2 * q2.x,
      w1 * this.y + w2 * q2.y,
      w1 * this.z + w2 * q2.z
    );
  }

  /**
   * Create a copy of this quaternion
   */
  clone(): Quaternion {
    return new Quaternion(this.w, this.x, this.y, this.z);
  }

  /**
   * Convert to array [w, x, y, z]
   */
  toArray(): number[] {
    return [this.w, this.x, this.y, this.z];
  }

  /**
   * Create quaternion from array [w, x, y, z]
   */
  static fromArray(arr: number[]): Quaternion {
    return new Quaternion(arr[0], arr[1], arr[2], arr[3]);
  }

  /**
   * String representation
   */
  toString(): string {
    return `Quaternion(w=${this.w.toFixed(4)}, x=${this.x.toFixed(4)}, y=${this.y.toFixed(4)}, z=${this.z.toFixed(4)})`;
  }
}
