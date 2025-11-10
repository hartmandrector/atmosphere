import { Filter } from './Filter';
import { Tensor1x3, Tensor3x1, Tensor3x3 } from '../utils/Tensor';

/**
 * Temperature Kalman Filter for estimating outside air temperature from sensor data.
 * 
 * This filter combines atmospheric model inversion (pressure + altitude -> temperature)
 * with device temperature bias learning to estimate the actual outside air temperature.
 * 
 * State vector (3D):
 *   [0] outside_air_temperature (K)
 *   [1] device_temperature_bias (K) - offset between device sensors and outside air
 *   [2] temperature_rate_of_change (K/s)
 */
export class FilterTemp implements Filter {
  private static readonly TAG = 'FilterTemp';

  // Physical constants for temperature estimation
  private static readonly GRAVITY = 9.80665;
  private static readonly GAS_CONSTANT = 8.31447;
  private static readonly LAPSE_RATE = 0.0065;
  private static readonly MM_AIR = 0.0289644;
  private static readonly TEMP_0 = 288.15;  // ISA standard temperature (K)
  private static readonly PRESSURE_0 = 101325.0;  // ISA standard pressure (Pa)
  
  // Barometric formula exponent
  private static readonly BARO_EXP = 
    FilterTemp.GRAVITY * FilterTemp.MM_AIR / (FilterTemp.GAS_CONSTANT * FilterTemp.LAPSE_RATE);

  // Filter parameters
  private readonly pressureMeasurementVariance: number;    // Variance in pressure measurements (Pa²)
  private readonly deviceTempVariance: number;             // Variance in device temperature measurements (K²)
  private readonly temperatureProcessVariance: number;     // How much outside temp can change (K²/s²)
  private readonly biasProcessVariance: number;           // How much device bias can change (K²/s²)
  private readonly rateProcessVariance: number;           // How much temp rate can change ((K/s)²/s²)
  private readonly tempOffset: number;                     // Temperature offset from ISA standard (K)

  // State vectors and matrices (3D)
  private readonly state: Tensor3x1 = new Tensor3x1(); // State estimate [temp, bias, rate]
  private readonly k: Tensor3x1 = new Tensor3x1(); // Kalman gain
  private readonly p: Tensor3x3 = new Tensor3x3(); // Error covariance
  private readonly q: Tensor3x3 = new Tensor3x3(); // Process noise covariance
  private readonly a: Tensor3x3 = new Tensor3x3(); // State transition matrix
  
  // Measurement matrices
  private readonly h_pressure: Tensor1x3 = new Tensor1x3(); // Pressure measurement model
  private readonly h_device: Tensor1x3 = new Tensor1x3();   // Device temperature measurement model
  private readonly temp_matrix: Tensor3x3 = new Tensor3x3(); // Temporary matrix for calculations

  // Filter state
  private static readonly INIT0 = 0; // No samples
  private static readonly INIT1 = 1; // First sample, initializing
  private static readonly READY = 2; // Ready for normal operation
  private filterState: number = FilterTemp.INIT0;

  // Current measurements for temperature calculation
  private currentPressure: number = FilterTemp.PRESSURE_0;
  private currentAltitude: number = 0.0;
  private deviceTemperature: number = FilterTemp.TEMP_0;
  private hasValidPressure: boolean = false;
  private hasValidDeviceTemp: boolean = false;

  constructor(
    pressureVar: number = 100.0,    // pressureMeasurementVariance (Pa²) - typical barometer accuracy
    deviceTempVar: number = 4.0,    // deviceTempVariance (K²) - device sensor accuracy  
    tempProcessVar: number = 0.01,  // temperatureProcessVariance (K²/s²) - slow natural temperature changes
    biasProcessVar: number = 0.001, // biasProcessVariance (K²/s²) - device bias changes slowly
    rateProcessVar: number = 0.1,   // rateProcessVariance ((K/s)²/s²) - temperature rate can change
    tempOffset: number = 10.0       // tempOffset (K) - temperature offset from ISA standard atmosphere
  ) {
    this.pressureMeasurementVariance = pressureVar;
    this.deviceTempVariance = deviceTempVar;
    this.temperatureProcessVariance = tempProcessVar;
    this.biasProcessVariance = biasProcessVar;
    this.rateProcessVariance = rateProcessVar;
    this.tempOffset = tempOffset;

    // Initialize measurement matrices
    this.h_pressure.set(1.0, 0.0, 0.0);  // Pressure measurement observes temperature directly
    this.h_device.set(1.0, 1.0, 0.0);    // Device measurement = outside temp + bias

    // Initialize process noise covariance
    this.q.set(0, 0, this.temperatureProcessVariance);
    this.q.set(1, 1, this.biasProcessVariance);
    this.q.set(2, 2, this.rateProcessVariance);

    // Initialize error covariance with high uncertainty
    this.p.set(0, 0, 100.0); // High initial uncertainty in temperature
    this.p.set(1, 1, 10.0);  // High initial uncertainty in bias
    this.p.set(2, 2, 1.0);   // High initial uncertainty in rate
  }

  /**
   * Update filter with pressure and altitude measurements
   * @param pressure Atmospheric pressure in Pa
   * @param altitude GPS altitude in meters
   * @param dt Time step in seconds
   */
  updatePressure(pressure: number, altitude: number, dt: number): void {
    this.currentPressure = pressure;
    this.currentAltitude = altitude;
    this.hasValidPressure = true;

    // Calculate temperature from atmospheric model
    const tempFromPressure = this.calculateTemperatureFromPressure(pressure, altitude);
    
    if (this.filterState === FilterTemp.INIT0) {
      // First pressure sample - initialize state
      this.state.set(tempFromPressure, 0.0, 0.0);
      this.filterState = FilterTemp.INIT1;
    } else if (dt > 0) {
      this.updateInternal(tempFromPressure, dt, true);
    }
  }

  /**
   * Update filter with device temperature measurement
   * @param deviceTemp Device internal temperature in K
   * @param dt Time step in seconds
   */
  updateDeviceTemperature(deviceTemp: number, dt: number): void {
    this.deviceTemperature = deviceTemp;
    this.hasValidDeviceTemp = true;

    if (this.filterState === FilterTemp.INIT0) {
      // Initialize with device temp (assume no bias initially)
      this.state.set(deviceTemp, 0.0, 0.0);
      this.filterState = FilterTemp.INIT1;
    } else if (dt > 0 && this.filterState !== FilterTemp.INIT0) {
      this.updateInternal(deviceTemp, dt, false);
    }
  }

  /**
   * Internal update method
   * @param measurement Temperature measurement in K
   * @param dt Time step in seconds  
   * @param isPressureMeasurement True if from pressure, false if from device sensor
   */
  private updateInternal(measurement: number, dt: number, isPressureMeasurement: boolean): void {
    // Prediction step
    // State transition: state = A * state
    // A = [1, 0, dt]
    //     [0, 1, 0 ]
    //     [0, 0, 1 ]
    this.a.setIdentity();
    this.a.set(0, 2, dt); // Temperature += rate * dt
    
    const x_pred = this.a.mult(this.state);
    
    // Covariance prediction: P = A * P * A^T + Q
    const ap = this.a.multMatrix(this.p);
    const apat = ap.multMatrix(this.transposeA(dt));
    const p_pred = apat.add(this.q.scale(dt));

    // Update step
    const h = isPressureMeasurement ? this.h_pressure : this.h_device;
    const r = isPressureMeasurement ? this.pressureMeasurementVariance : this.deviceTempVariance;

    // Innovation: y = z - H * x_pred
    const h_x = h.mult(x_pred);
    const innovation = measurement - h_x;

    // Innovation covariance: S = H * P_pred * H^T + R
    const hp = h.multMatrix(p_pred);
    const hpht = hp.mult(this.h_to_column(h));
    const s = hpht + r;

    // Kalman gain: K = P_pred * H^T / S
    const ph = this.p_mult_ht(p_pred, h);
    this.k.set(ph.get(0) / s, ph.get(1) / s, ph.get(2) / s);

    // State update: state = x_pred + K * innovation
    this.state.set(
      x_pred.get(0) + this.k.get(0) * innovation,
      x_pred.get(1) + this.k.get(1) * innovation,
      x_pred.get(2) + this.k.get(2) * innovation
    );

    // Covariance update: P = (I - K * H) * P_pred
    const kh = Tensor3x3.outerProduct(this.k, h);
    this.temp_matrix.setIdentity();
    const i_minus_kh = this.temp_matrix.subtract(kh);
    const p_new = i_minus_kh.multMatrix(p_pred);

    // Copy result back to p
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        this.p.set(i, j, p_new.get(i, j));
      }
    }

    // Mark as ready after first complete update
    if (this.filterState === FilterTemp.INIT1) {
      this.filterState = FilterTemp.READY;
    }
  }

  /**
   * Calculate outside air temperature from pressure and altitude using atmospheric model inversion.
   * 
   * The standard barometric formula is: P = P0 * ((T - L*h) / T0)^(g*M/(R*L))
   * where T is the sea-level temperature in the actual atmosphere.
   * 
   * We solve for T (the actual sea level temperature):
   * (P/P0)^(R*L/(g*M)) = (T - L*h) / T0
   * T = T0 * (P/P0)^(R*L/(g*M)) + L*h
   * 
   * Then the temperature at altitude h is: T_at_h = T - L*h
   * Combined: T_at_h = T0 * (P/P0)^(R*L/(g*M))
   * 
   * The tempOffset represents systematic deviation from ISA standard.
   */
  private calculateTemperatureFromPressure(pressure: number, altitude: number): number {
    // Avoid division by zero or invalid pressure
    if (pressure <= 0 || pressure > FilterTemp.PRESSURE_0 * 2) {
      return FilterTemp.TEMP_0 - FilterTemp.LAPSE_RATE * altitude + this.tempOffset;
    }
    
    // Calculate temperature at altitude using barometric formula inversion
    const pressureRatio = pressure / FilterTemp.PRESSURE_0;
    const exponent = (FilterTemp.GAS_CONSTANT * FilterTemp.LAPSE_RATE) / 
                     (FilterTemp.GRAVITY * FilterTemp.MM_AIR);
    
    // This gives the temperature at this altitude directly
    const tempAtAltitude = FilterTemp.TEMP_0 * Math.pow(pressureRatio, exponent);
    
    // Add the temperature offset to account for non-standard atmosphere
    const tempWithOffset = tempAtAltitude + this.tempOffset;

    return tempWithOffset;
  }

  // Helper methods for matrix operations
  private transposeA(dt: number): Tensor3x3 {
    const at = new Tensor3x3();
    at.setIdentity();
    at.set(2, 0, dt); // Transpose of A
    return at;
  }

  private h_to_column(h: Tensor1x3): Tensor3x1 {
    const col = new Tensor3x1();
    col.set(h.get(0), h.get(1), h.get(2));
    return col;
  }

  private p_mult_ht(p: Tensor3x3, h: Tensor1x3): Tensor3x1 {
    const result = new Tensor3x1();
    for (let i = 0; i < 3; i++) {
      let sum = 0;
      for (let j = 0; j < 3; j++) {
        sum += p.get(i, j) * h.get(j);
      }
      result.setAt(i, sum);
    }
    return result;
  }

  // Implement Filter interface methods
  update(z: number, dt: number): void {
    // Generic update - treat as device temperature
    this.updateDeviceTemperature(z, dt);
  }

  x(): number {
    return this.state.get(0);
  }

  v(): number {
    return this.state.get(2);
  }

  // Additional getters for temperature filter
  
  /**
   * Get estimated outside air temperature in Kelvin
   */
  getOutsideTemperature(): number {
    return this.state.get(0);
  }

  /**
   * Get estimated outside air temperature in Celsius
   */
  getOutsideTemperatureCelsius(): number {
    return this.state.get(0) - 273.15;
  }

  /**
   * Get estimated device temperature bias (device_temp - outside_temp)
   */
  getDeviceTemperatureBias(): number {
    return this.state.get(1);
  }

  /**
   * Get estimated temperature rate of change in K/s
   */
  getTemperatureRate(): number {
    return this.state.get(2);
  }

  /**
   * Get estimated outside temperature from current device reading
   */
  getOutsideTemperatureFromDevice(): number {
    if (!this.hasValidDeviceTemp) {
      return this.state.get(0);
    }
    return this.deviceTemperature - this.state.get(1);
  }

  /**
   * Check if filter has sufficient data for reliable estimates
   */
  isReady(): boolean {
    return this.filterState === FilterTemp.READY;
  }

  /**
   * Check if we have valid pressure data
   */
  hasValidPressureData(): boolean {
    return this.hasValidPressure;
  }

  /**
   * Check if we have valid device temperature data  
   */
  hasValidDeviceTemperatureData(): boolean {
    return this.hasValidDeviceTemp;
  }

  /**
   * Reset the filter to initial state
   */
  reset(): void {
    this.filterState = FilterTemp.INIT0;
    this.hasValidPressure = false;
    this.hasValidDeviceTemp = false;
    this.state.set(0, 0, 0);
    this.p.set(0, 0, 100.0);
    this.p.set(1, 1, 10.0);
    this.p.set(2, 2, 1.0);
  }

  /**
   * Get current filter uncertainty in temperature estimate (K)
   */
  getTemperatureUncertainty(): number {
    return Math.sqrt(Math.abs(this.p.get(0, 0)));
  }

  toString(): string {
    return `FilterTemp(temp=${this.getOutsideTemperatureCelsius().toFixed(2)}°C, bias=${this.getDeviceTemperatureBias().toFixed(2)}K, rate=${this.getTemperatureRate().toFixed(4)}K/s)`;
  }
}
