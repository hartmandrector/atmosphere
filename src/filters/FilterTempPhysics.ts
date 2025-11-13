import { Filter } from './Filter';
import { Tensor1x4, Tensor4x1, Tensor4x4 } from '../utils/Tensor';

/**
 * Physics-Based Temperature Kalman Filter
 * 
 * Uses thermal physics to estimate outside air temperature and device characteristics.
 * Device temperature dynamics: dT_device/dt = -α(v) × (T_device - T_outside) + β
 * 
 * State vector (4D):
 *   [0] outside_air_temperature (K) - estimated from barometric + dynamics
 *   [1] device_temperature_bias (K) - offset between device and outside
 *   [2] cooling_coefficient α (1/s) - estimated from temp dynamics, airspeed-dependent
 *   [3] temperature_rate dT_outside/dt (K/s) - estimated
 * 
 * User parameter (not in state):
 *   β = internal heating rate (K/s) - device constant, provided by slider
 * 
 * Derived quantity (computed, not in state):
 *   tempOffset = outside_temp - ISA_temp_at_altitude - shows deviation from standard atmosphere
 * 
 * Key improvement: Uses physics model to constrain estimates, providing automatic
 * temperature offset calculation without manual calibration.
 */
export class FilterTempPhysics implements Filter {
  private static readonly TAG = 'FilterTempPhysics';

  // Physical constants
  private static readonly GRAVITY = 9.80665;
  private static readonly GAS_CONSTANT = 8.31447;
  private static readonly LAPSE_RATE = 0.0065;
  private static readonly MM_AIR = 0.0289644;
  private static readonly TEMP_0 = 288.15;
  private static readonly PRESSURE_0 = 101325.0;
  
  private static readonly BARO_EXP = 
    FilterTempPhysics.GRAVITY * FilterTempPhysics.MM_AIR / 
    (FilterTempPhysics.GAS_CONSTANT * FilterTempPhysics.LAPSE_RATE);

  // Filter parameters
  private readonly pressureMeasurementVariance: number;
  private readonly deviceTempVariance: number;
  private readonly temperatureProcessVariance: number;
  private readonly biasProcessVariance: number;  // For device bias (state[1])
  private readonly coolingProcessVariance: number;  // For α (state[2])
  private readonly rateProcessVariance: number;  // For rate (state[3])
  private readonly heatingRate: number;  // β - user parameter, not estimated

  // State vectors and matrices (4D)
  private readonly state: Tensor4x1 = new Tensor4x1();
  private readonly k: Tensor4x1 = new Tensor4x1();
  private readonly p: Tensor4x4 = new Tensor4x4();
  private readonly q: Tensor4x4 = new Tensor4x4();
  private readonly a: Tensor4x4 = new Tensor4x4();
  
  // Measurement matrices
  private readonly h_pressure: Tensor1x4 = new Tensor1x4();
  private readonly h_device: Tensor1x4 = new Tensor1x4();
  private readonly h_dynamics: Tensor1x4 = new Tensor1x4();  // Observe temp dynamics
  private readonly temp_matrix: Tensor4x4 = new Tensor4x4();

  // Filter state
  private static readonly INIT0 = 0;
  private static readonly INIT1 = 1;
  private static readonly READY = 2;
  private filterState: number = FilterTempPhysics.INIT0;

  // Current measurements
  private currentPressure: number = FilterTempPhysics.PRESSURE_0;
  private currentAltitude: number = 0.0;
  private currentAirspeed: number = 0.0;
  private deviceTemperature: number = FilterTempPhysics.TEMP_0;
  private previousDeviceTemp: number = FilterTempPhysics.TEMP_0;
  private deviceTempRate: number = 0.0;  // dT_dev/dt computed from measurements
  private hasValidPressure: boolean = false;
  private hasValidDeviceTemp: boolean = false;
  private hasValidAirspeed: boolean = false;
  private hasValidTempRate: boolean = false;

  constructor(
    pressureVar: number = 100.0,
    deviceTempVar: number = 4.0,
    tempProcessVar: number = 0.01,
    biasProcessVar: number = 0.001,  // Device bias changes slowly
    coolingProcessVar: number = 0.01,  // α can vary with conditions
    rateProcessVar: number = 0.1,
    heatingRate: number = 0.5  // β - user provides this (K/s)
  ) {
    this.pressureMeasurementVariance = pressureVar;
    this.deviceTempVariance = deviceTempVar;
    this.temperatureProcessVariance = tempProcessVar;
    this.biasProcessVariance = biasProcessVar;
    this.coolingProcessVariance = coolingProcessVar;
    this.rateProcessVariance = rateProcessVar;
    this.heatingRate = heatingRate;

    // Initialize measurement matrices
    // State: [outside_temp, device_bias, α, rate]
    this.h_pressure.set(1.0, 0.0, 0.0, 0.0);  // Pressure observes: outside_temp
    this.h_device.set(1.0, 1.0, 0.0, 0.0);    // Device observes: outside_temp + device_bias

    // Initialize process noise covariance
    // State: [outside_temp, device_bias, α, rate]
    this.q.set(0, 0, this.temperatureProcessVariance);
    this.q.set(1, 1, this.biasProcessVariance);
    this.q.set(2, 2, this.coolingProcessVariance);
    this.q.set(3, 3, this.rateProcessVariance);

    // Initialize error covariance
    this.p.set(0, 0, 100.0);  // High uncertainty in outside temp
    this.p.set(1, 1, 10.0);   // Moderate uncertainty in device bias
    this.p.set(2, 2, 0.01);   // Cooling coefficient α
    this.p.set(3, 3, 1.0);    // Temperature rate
  }

  /**
   * Update with pressure and altitude
   */
  updatePressure(pressure: number, altitude: number, dt: number): void {
    this.currentPressure = pressure;
    this.currentAltitude = altitude;
    this.hasValidPressure = true;
  }

  /**
   * Update with device temperature
   */
  updateDeviceTemperature(temperature: number, dt: number): void {
    // Compute temperature rate from consecutive measurements
    if (this.hasValidDeviceTemp && dt > 0) {
      this.deviceTempRate = (temperature - this.previousDeviceTemp) / dt;
      this.hasValidTempRate = true;
    }
    
    this.previousDeviceTemp = this.deviceTemperature;
    this.deviceTemperature = temperature;
    this.hasValidDeviceTemp = true;
  }

  /**
   * Update with airspeed (velocity magnitude)
   */
  updateAirspeed(velN: number, velE: number, velD: number, dt: number): void {
    this.currentAirspeed = Math.sqrt(velN * velN + velE * velE + velD * velD);
    this.hasValidAirspeed = true;
  }

  /**
   * Process the measurements and update state
   */
  process(dt: number): void {
    if (this.filterState === FilterTempPhysics.INIT0) {
      this.initializeFilter();
      return;
    }

    if (this.filterState === FilterTempPhysics.INIT1) {
      this.filterState = FilterTempPhysics.READY;
    }

    // Prediction step with thermal physics
    this.predict(dt);

    // Update step with measurements
    if (this.hasValidPressure) {
      this.updateWithPressure();
      this.hasValidPressure = false;
    }

    if (this.hasValidDeviceTemp) {
      this.updateWithDeviceTemp();
      this.hasValidDeviceTemp = false;
    }
    
    // Update with temperature dynamics (makes α and β observable)
    if (this.hasValidTempRate && this.hasValidAirspeed) {
      this.updateWithTempDynamics(dt);
      this.hasValidTempRate = false;
    }
  }

  /**
   * Initialize filter with first measurements
   */
  private initializeFilter(): void {
    if (!this.hasValidDeviceTemp) return;

    // Get ISA temperature from pressure (without offset)
    const T_ISA = this.getTemperatureFromPressure(this.currentPressure);
    
    // Initial state estimate
    // State: [outside_temp, device_bias, α, rate]
    const outside_temp = T_ISA;  // Start with ISA
    const device_bias = 0.0;  // Initial bias unknown
    const alpha = 0.05;  // Initial cooling coefficient (1/s)
    const rate = 0.0;   // Initial temperature rate
    
    this.state.set(outside_temp, device_bias, alpha, rate);
    
    this.filterState = FilterTempPhysics.INIT1;
  }

  /**
   * Prediction step with physics-based thermal model
   */
  private predict(dt: number): void {
    // Extract state variables
    // State: [outside_temp, device_bias, α, rate]
    const T_out = this.state.get(0);
    const bias = this.state.get(1);
    const alpha = this.state.get(2);
    const rate = this.state.get(3);
    
    // β is a parameter, not in state
    const beta = this.heatingRate;

    // Adjust cooling coefficient based on airspeed
    const airspeedFactor = this.hasValidAirspeed ? 
      (1.0 + Math.sqrt(Math.max(0, this.currentAirspeed))) : 1.0;
    const alpha_effective = alpha * airspeedFactor;

    // Predict state using physics
    // Outside temp changes slowly
    const T_out_new = T_out + rate * dt;
    
    // Device bias and α are constant in prediction (change via process noise)
    const bias_new = bias;
    const alpha_new = alpha;
    const rate_new = rate;
    
    // Check for valid values
    if (isNaN(T_out_new)) {
      console.error('Invalid state in predict:', { T_out_new, rate });
      return;
    }
    
    // Update state
    this.state.set(T_out_new, bias_new, alpha_new, rate_new);

    // Build state transition matrix A
    // State: [outside_temp, device_bias, α, rate]
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        this.a.set(i, j, i === j ? 1.0 : 0.0);
      }
    }
    
    // T_out evolves with rate
    this.a.set(0, 3, dt);   // T_out += rate × dt
    
    // All other states (bias, α, rate) are constant in prediction
    // They change only through process noise and measurement updates

    // Update error covariance: P = A × P × A^T + Q
    
    // First: temp = A × P
    const temp = this.a.multiplyMatrix(this.p);
    
    // Second: P_new = temp × A^T
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        let sum = 0;
        for (let k = 0; k < 4; k++) {
          sum += temp.get(i, k) * this.a.get(j, k);  // A^T means swap indices
        }
        this.temp_matrix.set(i, j, sum);
      }
    }
    
    // Add process noise: P = A×P×A^T + Q
    const p_new = this.temp_matrix.add(this.q.scale(dt));
    
    // Copy to p with bounds checking and enforcement of symmetry
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        let val = p_new.get(i, j);
        
        // Enforce symmetry
        if (i !== j) {
          const val_sym = p_new.get(j, i);
          val = (val + val_sym) / 2.0;
        }
        
        // Check for NaN or Infinity and clamp to reasonable bounds
        if (isNaN(val) || !isFinite(val)) {
          console.error(`Invalid P[${i},${j}] = ${val}`);
          val = i === j ? 1.0 : 0.0;  // Reset to identity diagonal
        } else if (i === j && val < 1e-10) {
          val = 1e-10;  // Prevent diagonal from becoming zero
        } else if (i === j && val > 1000) {
          val = 1000;  // Clamp diagonal to prevent explosion
        }
        
        this.p.set(i, j, val);
      }
    }
  }

  /**
   * Update with pressure measurement
   */
  private updateWithPressure(): void {
    // Get expected temperature from barometric formula
    const z_measured = this.getTemperatureFromPressure(this.currentPressure);
    const z_predicted = this.h_pressure.multiply(this.state);
    const innovation = z_measured - z_predicted;

    // Calculate Kalman gain
    const s = this.h_pressure.multiplyMatrix(this.p).multiply(this.state) + 
              this.pressureMeasurementVariance;
    
    // K = P × H^T / s
    for (let i = 0; i < 4; i++) {
      let sum = 0;
      for (let j = 0; j < 4; j++) {
        sum += this.p.get(i, j) * this.h_pressure.get(j);
      }
      this.k.setAt(i, sum / s);
    }

    // Update state: x = x + K × innovation
    for (let i = 0; i < 4; i++) {
      this.state.setAt(i, this.state.get(i) + this.k.get(i) * innovation);
    }

    // Update covariance: P = (I - K × H) × P
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        const delta = this.k.get(i) * this.h_pressure.get(j);
        const newVal = this.p.get(i, j) - delta * this.p.get(i, j);
        this.p.set(i, j, newVal);
      }
    }
  }

  /**
   * Update with device temperature measurement
   */
  private updateWithDeviceTemp(): void {
    // deviceTemperature is already in Kelvin (converted in updateDeviceTemperature)
    const z_measured = this.deviceTemperature;
    const z_predicted = this.h_device.multiply(this.state);
    const innovation = z_measured - z_predicted;

    // Calculate Kalman gain
    const s = this.h_device.multiplyMatrix(this.p).multiply(this.state) + 
              this.deviceTempVariance;
    
    for (let i = 0; i < 4; i++) {
      let sum = 0;
      for (let j = 0; j < 4; j++) {
        sum += this.p.get(i, j) * this.h_device.get(j);
      }
      this.k.setAt(i, sum / s);
    }

    // Update state
    for (let i = 0; i < 4; i++) {
      this.state.setAt(i, this.state.get(i) + this.k.get(i) * innovation);
    }

    // Update covariance
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        const delta = this.k.get(i) * this.h_device.get(j);
        const newVal = this.p.get(i, j) - delta * this.p.get(i, j);
        this.p.set(i, j, newVal);
      }
    }
  }

  /**
   * Update with temperature dynamics measurement
   * Observes: dT_device/dt = -α(v) × bias + β
   * where bias = T_device - T_outside
   * 
   * This makes α observable! The rate device temp changes tells us the cooling rate.
   */
  private updateWithTempDynamics(dt: number): void {
    // Extract current state
    // State: [outside_temp, device_bias, α, rate]
    const T_out = this.state.get(0);
    const bias = this.state.get(1);
    const alpha = this.state.get(2);
    
    // β is a parameter
    const beta = this.heatingRate;
    
    // Calculate effective cooling with airspeed
    const airspeedFactor = this.hasValidAirspeed ? 
      (1.0 + Math.sqrt(Math.max(0, this.currentAirspeed))) : 1.0;
    const alpha_effective = alpha * airspeedFactor;
    
    // Measurement: observed dT_device/dt
    const z_measured = this.deviceTempRate;
    
    // Predicted: dT_device/dt = -α(v) × (T_device - T_outside) + β
    //                          = -α(v) × bias + β
    const z_predicted = -alpha_effective * bias + beta;
    const innovation = z_measured - z_predicted;
    
    // Build observation matrix H for this measurement
    // ∂(dT_device/dt)/∂state = [0, -α(v), -bias*airspeedFactor, 0]
    // State: [outside_temp, device_bias, α, rate]
    this.h_dynamics.set(
      0.0,                                  // ∂/∂T_out (doesn't affect rate directly)
      -alpha_effective,                     // ∂/∂bias (KEY: cooling depends on bias!)
      -bias * airspeedFactor,               // ∂/∂α (KEY: makes α observable!)
      0.0                                   // ∂/∂rate
    );
    
    // Measurement variance (temperature rate is noisy)
    const dynamics_variance = 1.0;  // K/s variance
    
    // Calculate Kalman gain: K = P × H^T / (H × P × H^T + R)
    const s = this.h_dynamics.multiplyMatrix(this.p).multiply(this.state) + dynamics_variance;
    
    for (let i = 0; i < 4; i++) {
      let sum = 0;
      for (let j = 0; j < 4; j++) {
        sum += this.p.get(i, j) * this.h_dynamics.get(j);
      }
      this.k.setAt(i, sum / s);
    }
    
    // Update state with innovation
    for (let i = 0; i < 4; i++) {
      const newVal = this.state.get(i) + this.k.get(i) * innovation;
      // Clamp α to reasonable range
      if (i === 2) {  // α
        this.state.setAt(i, Math.max(0.001, Math.min(0.5, newVal)));
      } else {
        this.state.setAt(i, newVal);
      }
    }
    
    // Update covariance: P = (I - K×H) × P
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        const delta = this.k.get(i) * this.h_dynamics.get(j);
        const newVal = this.p.get(i, j) - delta * this.p.get(i, j);
        this.p.set(i, j, newVal);
      }
    }
  }

  /**
   * Get ISA temperature from pressure using inverted barometric formula
   * Returns the ISA temperature (no offset - that's computed separately now)
   */
  private getTemperatureFromPressure(pressure: number): number {
    const ratio = pressure / FilterTempPhysics.PRESSURE_0;
    const exponent = 1.0 / FilterTempPhysics.BARO_EXP;
    const tempK = FilterTempPhysics.TEMP_0 * Math.pow(ratio, exponent);
    return tempK;
  }

  isReady(): boolean {
    return this.filterState === FilterTempPhysics.READY;
  }

  getOutsideTemperatureCelsius(): number {
    // State[0] is outside temp
    return this.state.get(0) - 273.15;
  }

  getDeviceTemperatureBias(): number {
    // State[1] is device bias
    return this.state.get(1);
  }

  getCoolingCoefficient(): number {
    // State[2] is α
    return this.state.get(2);
  }

  getTemperatureRate(): number {
    // State[3] is rate
    return this.state.get(3);
  }

  /**
   * Get temperature offset from ISA standard (computed, not in state)
   * Returns: estimated_outside_temp - ISA_temp_at_altitude
   */
  getTemperatureOffset(): number {
    if (!this.hasValidPressure) return 0;
    
    // Get ISA temperature at current altitude
    const T_ISA = FilterTempPhysics.TEMP_0 - FilterTempPhysics.LAPSE_RATE * this.currentAltitude;
    
    // Get estimated outside temperature
    const T_out = this.state.get(0);
    
    // Offset is the difference
    return T_out - T_ISA;
  }

  // Helper: get heating rate (β is a parameter, not in state)
  getHeatingRateParameter(): number {
    return this.heatingRate;
  }

  getUncertainty(): number {
    return Math.sqrt(this.p.get(0, 0));
  }

  // Filter interface methods
  update(z: number, dt: number): void {
    this.updateDeviceTemperature(z, dt);
    this.process(dt);
  }

  x(): number {
    return this.getOutsideTemperatureCelsius();
  }

  v(): number {
    return this.getTemperatureRate();
  }
}
