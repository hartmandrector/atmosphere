/**
 * Base interface for filters
 */
export interface Filter {
  /**
   * Update filter with a measurement
   * @param z Measurement value
   * @param dt Time step in seconds
   */
  update(z: number, dt: number): void;

  /**
   * Get current state estimate
   */
  x(): number;

  /**
   * Get current velocity/rate estimate
   */
  v(): number;
}
