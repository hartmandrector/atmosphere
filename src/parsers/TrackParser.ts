import * as fs from 'fs';

export interface TrackPoint {
  millis: number;      // Timestamp in milliseconds
  latitude: number;    // Latitude in degrees
  longitude: number;   // Longitude in degrees
  altitude: number;    // Altitude in meters (MSL)
  velN: number;        // North velocity in m/s
  velE: number;        // East velocity in m/s
  velD: number;        // Down velocity in m/s
  hAccuracy: number;   // Horizontal accuracy in meters
  vAccuracy: number;   // Vertical accuracy in meters
  sAccuracy: number;   // Speed accuracy in m/s
  satellites: number;  // Number of satellites
}

/**
 * Parse TRACK.CSV file containing GPS data in custom $GNSS format
 */
export class TrackParser {
  private data: TrackPoint[] = [];

  /**
   * Load and parse a TRACK.CSV file
   * @param filePath Path to the TRACK.CSV file
   */
  load(filePath: string): void {
    const content = fs.readFileSync(filePath, 'utf-8');
    this.loadFromString(content);
  }

  /**
   * Parse TRACK.CSV content from a string
   * @param content CSV file content as string
   */
  loadFromString(content: string): void {
    this.data = []; // Clear existing data
    const lines = content.split(/\r?\n/);

    for (const line of lines) {
      if (line.startsWith('$GNSS,')) {
        const parts = line.substring(6).split(',');
        if (parts.length >= 11) {
          // Parse ISO timestamp to milliseconds
          const timestamp = new Date(parts[0]).getTime();
          
          this.data.push({
            millis: timestamp,
            latitude: parseFloat(parts[1]),
            longitude: parseFloat(parts[2]),
            altitude: parseFloat(parts[3]),
            velN: parseFloat(parts[4]),
            velE: parseFloat(parts[5]),
            velD: parseFloat(parts[6]),
            hAccuracy: parseFloat(parts[7]),
            vAccuracy: parseFloat(parts[8]),
            sAccuracy: parseFloat(parts[9]),
            satellites: parseInt(parts[10])
          });
        }
      }
    }
  }

  /**
   * Get all track points
   */
  getData(): TrackPoint[] {
    return this.data;
  }

  /**
   * Get track point at specific index
   */
  getPoint(index: number): TrackPoint | undefined {
    return this.data[index];
  }

  /**
   * Get number of track points
   */
  getCount(): number {
    return this.data.length;
  }

  /**
   * Get track duration in seconds
   */
  getDuration(): number {
    if (this.data.length < 2) return 0;
    return (this.data[this.data.length - 1].millis - this.data[0].millis) / 1000;
  }
}
