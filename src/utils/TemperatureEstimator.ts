/**
 * Estimates outside air temperature offset from ISA based on GPS location and time.
 * Uses latitude, longitude, day of year, and solar time to model temperature variations.
 * 
 * The model accounts for:
 * - Latitude effect (warmer at equator)
 * - Seasonal variation (warmer in summer)
 * - Diurnal (daily) variation (warmer in afternoon)
 *
 * Ported from BASElineXR Kotlin implementation.
 */

export interface TemperatureEstimatorParams {
  T_EQUATOR: number;      // °C - base temperature at equator (sea level)
  A_LAT: number;          // °C - latitude amplitude for sin²(lat) term
  A_YEAR: number;         // K - yearly/seasonal amplitude
  A_DAY: number;          // K - daily amplitude
  H0: number;             // base lag (hours)
  ALPHA: number;          // latitude lag effect (hours)
  BETA: number;           // seasonal lag effect (hours)
  LAPSE_RATE: number;     // °C per km - temperature decrease with altitude
  SUMMER_PEAK_DAY: number | null; // day of year for peak summer temp, or null to auto-calculate from latitude
}

export interface TemperatureEstimateResult {
  offsetK: number;           // Temperature offset from ISA in Kelvin
  offsetC: number;           // Temperature offset from ISA in Celsius (same as K for delta)
  offsetF: number;           // Temperature offset from ISA in Fahrenheit
  seaLevelTempC: number;     // Estimated sea level temperature in Celsius
  seaLevelTempF: number;     // Estimated sea level temperature in Fahrenheit
  altitudeTempC: number;     // Estimated temperature at altitude in Celsius
  altitudeTempF: number;     // Estimated temperature at altitude in Fahrenheit
  solarHour: number;         // Local solar hour (0-24)
  dayOfYear: number;         // Day of year (1-365)
  lag: number;               // Peak temperature lag in hours
  localDateTime: string;     // Formatted local date/time string
  utcDateTime: string;       // Formatted UTC date/time string
}

/**
 * TEMPERATURE MODEL FORMULA
 * ==========================
 * 
 * T = T_base + A_YEAR × cos(seasonal) + A_DAY × cos(diurnal) - LAPSE_RATE × altitude
 * 
 * Where:
 *   T_base = T_EQUATOR - A_LAT × sin²(latitude)
 *   
 *   seasonal = 2π × (dayOfYear - summerPeakDay) / 365
 *   diurnal  = 2π × (solarHour - 12 - lag) / 24
 *   
 *   lag = H0 + ALPHA × (1 - cos(latitude)) + BETA × cos(2π × (dayOfYear - 172) / 365)
 *   
 *   summerPeakDay = blend × northPeak + (1-blend) × southPeak
 *     where blend = 0.5 × (1 + tanh(latitude/5))  [smooth hemisphere transition]
 *     northPeak = 172 + 30 × (1 - cos(|lat| × π/90))
 *     southPeak = northPeak + 183 (6 months offset)
 *   
 *   solarHour = UTC_hour + longitude/15 + EoT/60  [Equation of Time correction]
 * 
 * Note: The tanh blending ensures smooth transition across the equator,
 *       avoiding discontinuity when switching between hemisphere seasons.
 * 
 * Units:
 *   T, T_base, T_EQUATOR, A_LAT, A_YEAR, A_DAY: °C (or K for amplitudes)
 *   LAPSE_RATE: °C/km
 *   altitude: km
 *   latitude, longitude: degrees
 *   lag, H0, ALPHA, BETA: hours
 *   solarHour: hours (0-24)
 *   dayOfYear: days (1-365)
 * 
 * Default Parameters (Continental - Ogden, UT):
 *   T_EQUATOR = 32°C, A_LAT = 23°C, A_YEAR = 16 K, A_DAY = 10 K
 *   H0 = 3.0 hr, ALPHA = 1.2 hr, BETA = 1.0 hr, LAPSE_RATE = 6.5°C/km
 * 
 * =============================================================================
 * 
 * TUNING GUIDE
 * =============
 * 
 * STABLE PARAMETERS (keep as defaults):
 *   A_LAT = 23°C        - Latitude gradient (equator-to-pole temperature difference)
 *   LAPSE_RATE = 6.5    - °C per km (ISA standard)
 *   SUMMER_PEAK_DAY     - Auto-calculated from latitude (null = auto)
 * 
 * LAG PARAMETERS (hours after solar noon for peak temperature):
 *   Formula: lag = H0 + ALPHA × sin²(lat) + BETA × cos(seasonAngle)
 * 
 *   H0 = 3.0            - Base lag (hours) - stable default
 *   ALPHA = 1.2         - Latitude effect (hours) - stable default
 *   BETA = 1.0          - Seasonal effect (hours) - stable default
 * 
 *   Observed peak times (Aug 20, 2024):
 *     Bolzano (46.5°N, alpine):     4:50 PM = 4.8 hrs (longest - valley heat trap)
 *     Ogden (41.2°N, continental):  3:53 PM = 3.9 hrs
 *     Seattle (47.4°N, maritime):   3:56 PM = 3.9 hrs
 *     Trondheim (63.5°N, coastal):  3:20 PM = 3.3 hrs (shorter than model - coastal)
 *     Cape Town (33.9°S, coastal):  2:47 PM = 2.8 hrs (winter + coastal)
 *     Kunming (25.0°N, monsoon):    2:00 PM = 2.0 hrs (afternoon clouds)
 * 
 *   Notes:
 *     - Model defaults work well for continental/alpine climates
 *     - Coastal climates: effective lag ~0.5-1 hr shorter (ocean moderation)
 *     - Monsoon/plateau: lag can be 1-2 hrs shorter (afternoon cloudiness)
 *     - To reduce lag: lower H0 or ALPHA
 * 
 * TUNABLE PARAMETERS (vary by climate type):
 * 
 *   T_EQUATOR - Base temperature / Continentality factor
 *     Continental (Ogden, inland US):  32°C (default)
 *     Alpine (Bolzano, Swiss Alps):    26°C
 *     Maritime (Seattle, coastal):     26-27°C
 *     Coastal (Trondheim, Cape Town):  25°C
 *     → Lower by 5-7°C for maritime/coastal climates
 * 
 *   A_YEAR - Seasonal amplitude (summer-winter swing)
 *     Continental:    16 K (default) - large seasonal swing
 *     Alpine:         11-12 K
 *     Maritime:       8-9 K
 *     Mediterranean:  5-6 K - small seasonal swing
 * 
 *   A_DAY - Daily amplitude (day-night swing)
 *     Continental:    10 K (default) - large daily swing
 *     Alpine:         7-8 K
 *     Maritime:       5-6 K
 *     Coastal:        4-5 K - small daily swing
 * 
 * USER TUNING WORKFLOW:
 *   1. Set location (lat/lon/alt) - latitude effect is automatic
 *   2. If coastal/maritime: Lower T_EQUATOR by 5-7°C
 *   3. Adjust A_YEAR to match observed seasonal swing
 *   4. Optionally adjust A_DAY for daily swing
 *   5. If peak temp occurs earlier than model: reduce H0 by 0.5-1 hr
 */

export class TemperatureEstimator {
  // Default model parameters (tuned for continental climate - Ogden, UT)
  private params: TemperatureEstimatorParams = {
    T_EQUATOR: 32.0,       // °C - base temperature at equator (lower for maritime)
    A_LAT: 23.0,           // °C - latitude amplitude for sin²(lat) term
    A_YEAR: 16.0,          // K - yearly/seasonal amplitude (reduce for maritime)
    A_DAY: 10.0,           // K - daily amplitude (reduce for coastal)
    H0: 3.0,               // base lag (hours)
    ALPHA: 1.2,            // latitude lag effect (hours)
    BETA: 1.0,             // seasonal lag effect (hours)
    LAPSE_RATE: 6.5,       // °C per km - ISA standard lapse rate
    SUMMER_PEAK_DAY: null  // null = auto-calculate from latitude
  };

  constructor(params?: Partial<TemperatureEstimatorParams>) {
    if (params) {
      this.params = { ...this.params, ...params };
    }
  }

  /**
   * Update model parameters
   */
  setParams(params: Partial<TemperatureEstimatorParams>): void {
    this.params = { ...this.params, ...params };
  }

  /**
   * Get current model parameters
   */
  getParams(): TemperatureEstimatorParams {
    return { ...this.params };
  }

  /**
   * Estimate temperature offset from raw parameters.
   * @param latitude Latitude in degrees
   * @param longitude Longitude in degrees
   * @param gpsTimeMillis GPS time in milliseconds (UTC)
   * @param altitudeMeters Altitude in meters (default 0 for sea level)
   * @returns Comprehensive temperature estimate result
   */
  estimateTemperatureOffset(latitude: number, longitude: number, gpsTimeMillis: number, altitudeMeters: number = 0): TemperatureEstimateResult {
    const { T_EQUATOR, A_LAT, A_YEAR, A_DAY, H0, ALPHA, BETA, LAPSE_RATE, SUMMER_PEAK_DAY } = this.params;

    // Convert GPS time to day of year and hour (GPS time is essentially UTC)
    const date = new Date(gpsTimeMillis);
    
    const dayOfYear = this.getDayOfYear(date);
    const utcHour = date.getUTCHours() + date.getUTCMinutes() / 60.0;

    // Calculate local solar time from UTC and longitude
    // Each 15° of longitude = 1 hour offset from UTC
    // Equation of Time correction for Earth's orbital eccentricity
    const B = 2 * Math.PI * (dayOfYear - 81) / 365.0;
    const EoT = 9.87 * Math.sin(2 * B) - 7.53 * Math.cos(B) - 1.5 * Math.sin(B); // minutes

    // Solar time (hours) - longitude directly gives offset from UTC
    const solarHour = utcHour + (longitude / 15.0) + EoT / 60.0;

    // Normalize solar hour to 0-24
    const normalizedSolarHour = ((solarHour % 24.0) + 24.0) % 24.0;

    // Lag function (hours) - peak temperature occurs after solar noon
    // Use solstice (day 172) for lag calculation as it reflects solar geometry
    const lag = H0 + ALPHA * (1 - Math.cos(this.toRadians(latitude))) +
                BETA * Math.cos(2 * Math.PI * (dayOfYear - 172) / 365.0);

    // Latitude effect using sin²(latitude) formula
    // T_surface = T_EQUATOR - A_LAT × sin²(latitude)
    // With defaults (26°C, 46°C): ~26°C at equator, ~3°C at 45°, ~-20°C at poles
    const sinLat = Math.sin(this.toRadians(latitude));
    const latitudeBaseTempC = T_EQUATOR - A_LAT * sinLat * sinLat;

    // Calculate summer peak day - either use override or auto-calculate from latitude
    // Thermal lag increases with latitude: equator ~172 (solstice), mid-lat ~204, arctic ~220+
    // Formula: 172 + 30 × (1 - cos(lat × π/90)) for Northern Hemisphere
    // For Southern Hemisphere, peak is ~6 months offset (add 183 days)
    const absLat = Math.abs(latitude);
    const autoSummerPeakDay = 172 + 30 * (1 - Math.cos(this.toRadians(absLat) * 2));
    const effectiveSummerPeakDay = SUMMER_PEAK_DAY !== null ? SUMMER_PEAK_DAY : autoSummerPeakDay;
    
    // For Southern Hemisphere, shift by 6 months
    // Use smooth blending near equator to avoid discontinuity at lat=0
    const northernPeakDay = effectiveSummerPeakDay;
    const southernPeakDay = (effectiveSummerPeakDay + 183) % 365;
    
    // Blend factor: 1.0 in Northern Hemisphere, 0.0 in Southern, smooth transition near equator
    // Using tanh for smooth transition over ~10° band centered on equator
    const blendFactor = 0.5 * (1.0 + Math.tanh(latitude / 5.0));
    
    // Interpolate between northern and southern peak days
    // This gives a smooth transition across the equator
    const hemisphereAdjustedPeakDay = blendFactor * northernPeakDay + (1.0 - blendFactor) * southernPeakDay;

    // Seasonal and diurnal modulation
    // Seasonal term peaks at summer peak day (thermal lag after solstice)
    // Diurnal term: peak at solar noon + lag (typically 14:00-15:00 solar time)
    const seasonalTerm = A_YEAR * Math.cos(2 * Math.PI * (dayOfYear - hemisphereAdjustedPeakDay) / 365.0);
    const diurnalTerm = A_DAY * Math.cos(2 * Math.PI * (normalizedSolarHour - 12.0 - lag) / 24.0);

    // Calculate sea level temperature
    const seaLevelTempC = latitudeBaseTempC + seasonalTerm + diurnalTerm;
    const seaLevelTempF = seaLevelTempC * 9.0 / 5.0 + 32.0;

    // Calculate temperature at altitude using lapse rate
    // Lapse rate is in °C per km, altitude is in meters
    const altitudeTempC = seaLevelTempC - (LAPSE_RATE * altitudeMeters / 1000.0);
    const altitudeTempF = altitudeTempC * 9.0 / 5.0 + 32.0;

    // Calculate offset from ISA standard (15°C at sea level)
    const isaSeaLevelTemp = 15.0; // ISA standard temperature at sea level in °C
    const deltaT = seaLevelTempC - isaSeaLevelTemp;

    // Format date/time strings
    const localOffsetHours = Math.round(longitude / 15.0);
    const localDate = new Date(gpsTimeMillis + localOffsetHours * 3600000);
    
    const utcDateTime = this.formatDateTime(date, 'UTC');
    const localDateTime = this.formatDateTime(localDate, `UTC${localOffsetHours >= 0 ? '+' : ''}${localOffsetHours}`);

    return {
      offsetK: deltaT,
      offsetC: deltaT,
      offsetF: deltaT * 9.0 / 5.0,
      seaLevelTempC,
      seaLevelTempF,
      altitudeTempC,
      altitudeTempF,
      solarHour: normalizedSolarHour,
      dayOfYear,
      lag,
      localDateTime,
      utcDateTime
    };
  }

  /**
   * Estimate the air temperature at a given altitude based on GPS location and time.
   * Convenience method that calls estimateTemperatureOffset with altitude.
   * 
   * @param latitude Latitude in degrees
   * @param longitude Longitude in degrees
   * @param gpsTimeMillis GPS time in milliseconds
   * @param altitudeMeters Altitude in meters
   * @returns Temperature in Celsius at the specified altitude
   */
  estimateTemperatureAtAltitude(latitude: number, longitude: number, gpsTimeMillis: number, altitudeMeters: number): number {
    const result = this.estimateTemperatureOffset(latitude, longitude, gpsTimeMillis, altitudeMeters);
    return result.altitudeTempC;
  }

  /**
   * Get day of year (1-365/366)
   */
  private getDayOfYear(date: Date): number {
    const start = new Date(date.getUTCFullYear(), 0, 0);
    const diff = date.getTime() - start.getTime();
    const oneDay = 1000 * 60 * 60 * 24;
    return Math.floor(diff / oneDay);
  }

  /**
   * Convert degrees to radians
   */
  private toRadians(degrees: number): number {
    return degrees * Math.PI / 180;
  }

  /**
   * Format date/time as string
   */
  private formatDateTime(date: Date, timezone: string): string {
    const year = date.getUTCFullYear();
    const month = String(date.getUTCMonth() + 1).padStart(2, '0');
    const day = String(date.getUTCDate()).padStart(2, '0');
    const hours = String(date.getUTCHours()).padStart(2, '0');
    const minutes = String(date.getUTCMinutes()).padStart(2, '0');
    const seconds = String(date.getUTCSeconds()).padStart(2, '0');
    
    return `${year}-${month}-${day} ${hours}:${minutes}:${seconds} ${timezone}`;
  }

  /**
   * Generate a daily temperature profile for debugging/visualization
   */
  generateDailyProfile(latitude: number, longitude: number, gpsTimeMillis: number): Array<{
    localHour: number;
    utcHour: number;
    solarHour: number;
    offsetC: number;
    offsetF: number;
    seaLevelTempF: number;
  }> {
    const profile: Array<{
      localHour: number;
      utcHour: number;
      solarHour: number;
      offsetC: number;
      offsetF: number;
      seaLevelTempF: number;
    }> = [];

    const localOffsetHours = Math.round(longitude / 15.0);

    for (let localHour = 0; localHour <= 23; localHour++) {
      // Calculate what UTC hour corresponds to this local hour
      const utcHour = (localHour - localOffsetHours + 24) % 24;

      // Create a GPS timestamp for this hour (same day as the GPS fix)
      const testDate = new Date(gpsTimeMillis);
      testDate.setUTCHours(utcHour, 0, 0, 0);
      const testGpsTimeMillis = testDate.getTime();

      const result = this.estimateTemperatureOffset(latitude, longitude, testGpsTimeMillis, 0);

      profile.push({
        localHour,
        utcHour,
        solarHour: result.solarHour,
        offsetC: result.offsetC,
        offsetF: result.offsetF,
        seaLevelTempF: result.seaLevelTempF
      });
    }

    return profile;
  }
}
