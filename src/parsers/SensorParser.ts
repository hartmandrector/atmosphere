import * as fs from 'fs';

export interface SensorData {
  millis: number;           // Timestamp in milliseconds (GPS time)
  pressure?: number;        // Atmospheric pressure in Pa (from BARO)
  temperature?: number;     // Temperature in Celsius (from BARO, HUM, MAG, or IMU)
  tempSource?: 'BARO' | 'HUM' | 'MAG' | 'IMU';  // Which sensor provided the temperature
  humidity?: number;        // Relative humidity (0-100) (from HUM)
  wx?: number;              // Angular velocity X (rad/s) (from IMU)
  wy?: number;              // Angular velocity Y (rad/s) (from IMU)
  wz?: number;              // Angular velocity Z (rad/s) (from IMU)
  ax?: number;              // Acceleration X (m/s²) (from IMU)
  ay?: number;              // Acceleration Y (m/s²) (from IMU)
  az?: number;              // Acceleration Z (m/s²) (from IMU)
  mx?: number;              // Magnetic field X (gauss) (from MAG)
  my?: number;              // Magnetic field Y (gauss) (from MAG)
  mz?: number;              // Magnetic field Z (gauss) (from MAG)
  baroTemp?: number;        // Temperature from BARO sensor (Celsius)
  humTemp?: number;         // Temperature from HUM sensor (Celsius)
  magTemp?: number;         // Temperature from MAG sensor (Celsius)
  imuTemp?: number;         // Temperature from IMU sensor (Celsius)
}

/**
 * GPS time synchronization info from $TIME entries
 */
interface TimeSync {
  sensorTime: number;    // Sensor clock time (seconds)
  gpsWeek: number;       // GPS week number
  timeOfWeek: number;    // GPS time of week (seconds)
  gpsEpochMs: number;    // Calculated GPS epoch time in milliseconds
}

/**
 * Parse SENSOR.CSV file containing sensor data in custom format
 * Supports $BARO, $HUM, $MAG, $IMU, and $TIME entries
 * 
 * Time Synchronization:
 * - Sensor data uses a local clock with arbitrary epoch (time column)
 * - $TIME entries provide GPS week/timeOfWeek synchronization points
 * - All sensor timestamps are converted to GPS epoch time (milliseconds since 1980-01-06)
 * - This allows sensor data to be merged with GPS TRACK data on a common timeline
 */
export class SensorParser {
  private data: SensorData[] = [];
  private timeSync: TimeSync | null = null;
  
  // GPS epoch: January 6, 1980 00:00:00 UTC
  private static readonly GPS_EPOCH_MS = new Date('1980-01-06T00:00:00Z').getTime();

  /**
   * Load and parse a SENSOR.CSV file
   * @param filePath Path to the SENSOR.CSV file
   */
  load(filePath: string): void {
    const content = fs.readFileSync(filePath, 'utf-8');
    this.loadFromString(content);
  }

  /**
   * Parse SENSOR.CSV content from a string
   * @param content CSV file content as string
   */
  loadFromString(content: string): void {
    this.data = []; // Clear existing data
    this.timeSync = null; // Reset time sync
    const lines = content.split(/\r?\n/);

    for (const line of lines) {
      const trimmed = line.trim();
      
      if (trimmed.startsWith('$TIME,')) {
        // $TIME,time,tow,week
        // time = sensor clock time (decimal seconds)
        // tow = GPS time of week (integer seconds)
        // week = GPS week number (integer)
        const parts = trimmed.substring(6).split(',');
        if (parts.length >= 3) {
          const sensorTime = parseFloat(parts[0]);
          const timeOfWeek = parseFloat(parts[1]);
          const gpsWeek = parseInt(parts[2]);
          
          // Calculate GPS epoch time from week and time of week
          // GPS time = weeks * 604800 + time_of_week (in seconds)
          const gpsSeconds = gpsWeek * 604800 + timeOfWeek;
          const gpsEpochMs = SensorParser.GPS_EPOCH_MS + gpsSeconds * 1000;
          
          this.timeSync = {
            sensorTime,
            gpsWeek,
            timeOfWeek,
            gpsEpochMs
          };
        }
      } else if (this.timeSync !== null) {
        // Only process sensor data after we have a $TIME sync
        
        if (trimmed.startsWith('$BARO,')) {
          // $BARO,time,pressure,temperature
          const parts = trimmed.substring(6).split(',');
          if (parts.length >= 3) {
            this.data.push({
              millis: this.convertSensorTimeToGPS(parseFloat(parts[0])),
              pressure: parseFloat(parts[1]),
              baroTemp: parseFloat(parts[2])
            });
          }
        } else if (trimmed.startsWith('$HUM,')) {
          // $HUM,time,humidity,temperature
          const parts = trimmed.substring(5).split(',');
          if (parts.length >= 3) {
            this.data.push({
              millis: this.convertSensorTimeToGPS(parseFloat(parts[0])),
              humidity: parseFloat(parts[1]),
              humTemp: parseFloat(parts[2])
            });
          }
        } else if (trimmed.startsWith('$MAG,')) {
          // $MAG,time,x,y,z,temperature
          const parts = trimmed.substring(5).split(',');
          if (parts.length >= 5) {
            this.data.push({
              millis: this.convertSensorTimeToGPS(parseFloat(parts[0])),
              mx: parseFloat(parts[1]),
              my: parseFloat(parts[2]),
              mz: parseFloat(parts[3]),
              magTemp: parseFloat(parts[4])
            });
          }
        } else if (trimmed.startsWith('$IMU,')) {
          // $IMU,time,wx,wy,wz,ax,ay,az,temperature
          const parts = trimmed.substring(5).split(',');
          if (parts.length >= 8) {
            this.data.push({
              millis: this.convertSensorTimeToGPS(parseFloat(parts[0])),
              wx: parseFloat(parts[1]),
              wy: parseFloat(parts[2]),
              wz: parseFloat(parts[3]),
              ax: parseFloat(parts[4]),
              ay: parseFloat(parts[5]),
              az: parseFloat(parts[6]),
              imuTemp: parseFloat(parts[7])
            });
          }
        }
      }
    }

    // Sort by timestamp
    this.data.sort((a, b) => a.millis - b.millis);
  }
  
  /**
   * Convert sensor clock time to GPS epoch time in milliseconds
   * Uses the most recent $TIME sync point
   */
  private convertSensorTimeToGPS(sensorTime: number): number {
    if (!this.timeSync) {
      return sensorTime * 1000; // Fallback if no sync available
    }
    
    // Calculate offset from the sync point
    const sensorDelta = sensorTime - this.timeSync.sensorTime;
    
    // Apply the same delta to GPS time (converting to milliseconds)
    return this.timeSync.gpsEpochMs + (sensorDelta * 1000);
  }

  /**
   * Get all sensor data points
   */
  getData(): SensorData[] {
    return this.data;
  }

  /**
   * Get sensor data at specific index
   */
  getPoint(index: number): SensorData | undefined {
    return this.data[index];
  }

  /**
   * Get number of sensor data points
   */
  getCount(): number {
    return this.data.length;
  }

  /**
   * Get sensor data duration in seconds
   */
  getDuration(): number {
    if (this.data.length < 2) return 0;
    return (this.data[this.data.length - 1].millis - this.data[0].millis) / 1000;
  }
}
