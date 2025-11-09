import * as path from 'path';
import { FilterTemp } from './filters/FilterTemp';
import { TrackParser } from './parsers/TrackParser';
import { SensorParser } from './parsers/SensorParser';

/**
 * Main entry point for testing the temperature filter
 */
function main() {
  // Initialize parsers
  const trackParser = new TrackParser();
  const sensorParser = new SensorParser();

  // Load default data files
  const trackPath = path.join(__dirname, '../public/data/TRACK.CSV');
  const sensorPath = path.join(__dirname, '../public/data/SENSOR.CSV');

  console.log('Loading data files...');
  trackParser.load(trackPath);
  sensorParser.load(sensorPath);

  console.log(`Loaded ${trackParser.getCount()} GPS points`);
  console.log(`Loaded ${sensorParser.getCount()} sensor readings`);
  
  // Show time ranges
  const trackData = trackParser.getData();
  const sensorData = sensorParser.getData();
  if (trackData.length > 0) {
    const trackStart = new Date(trackData[0].millis);
    const trackEnd = new Date(trackData[trackData.length - 1].millis);
    console.log(`GPS time range: ${trackStart.toISOString()} to ${trackEnd.toISOString()}`);
  }
  if (sensorData.length > 0) {
    const sensorStart = new Date(sensorData[0].millis);
    const sensorEnd = new Date(sensorData[sensorData.length - 1].millis);
    console.log(`Sensor time range: ${sensorStart.toISOString()} to ${sensorEnd.toISOString()}`);
  }

  // Initialize temperature filter
  const filter = new FilterTemp();

  console.log('\nProcessing data through temperature filter...');
  console.log('Time(s),Pressure(hPa),Device_Temp(C),Filtered_Temp(C),Bias(K),Rate(K/s),Uncertainty(K)');

  let trackIndex = 0;
  let sensorIndex = 0;
  let lastTime = 0;

  // Merge and process data streams
  while (trackIndex < trackData.length || sensorIndex < sensorData.length) {
    let nextTrack = trackIndex < trackData.length ? trackData[trackIndex] : null;
    let nextSensor = sensorIndex < sensorData.length ? sensorData[sensorIndex] : null;

    let currentTime: number;
    let dt: number;

    // Process whichever comes first
    if (nextTrack && (!nextSensor || nextTrack.millis <= nextSensor.millis)) {
      currentTime = nextTrack.millis / 1000;
      dt = lastTime > 0 ? currentTime - lastTime : 0;

      // For GPS data, we need barometric data to update pressure
      // Skip track-only updates for now since we need pressure from BARO sensor
      
      trackIndex++;
    } else if (nextSensor) {
      currentTime = nextSensor.millis / 1000;
      dt = lastTime > 0 ? currentTime - lastTime : 0;

      // Update from barometric sensor (has both pressure and temperature)
      if (nextSensor.pressure && nextSensor.temperature !== undefined) {
        // We have pressure but need altitude from GPS
        // Find closest GPS point
        let closestAltitude = 0;
        for (let i = Math.max(0, trackIndex - 10); i < Math.min(trackData.length, trackIndex + 10); i++) {
          if (Math.abs(trackData[i].millis - nextSensor.millis) < 1000) {
            closestAltitude = trackData[i].altitude;
            break;
          }
        }
        
        if (closestAltitude !== 0 && dt > 0) {
          filter.updatePressure(nextSensor.pressure, closestAltitude, dt);
        }
        
        // Also update with device temperature (convert C to K)
        if (nextSensor.temperature !== undefined && dt > 0) {
          filter.updateDeviceTemperature(nextSensor.temperature + 273.15, dt);
        }
      } else if (nextSensor.temperature !== undefined && dt > 0) {
        // Temperature-only measurement (convert C to K)
        filter.updateDeviceTemperature(nextSensor.temperature + 273.15, dt);
      }

      sensorIndex++;
    } else {
      break;
    }

    lastTime = currentTime;

    // Output results every 10 seconds
    if (filter.isReady() && Math.floor(currentTime * 10) % 100 === 0) {
      const deviceTemp = nextSensor && nextSensor.temperature !== undefined ? 
        nextSensor.temperature.toFixed(2) : 'N/A';
      const pressure = nextSensor && nextSensor.pressure ? 
        (nextSensor.pressure / 100).toFixed(2) : 'N/A'; // Convert to hPa for display
      
      console.log(
        `${currentTime.toFixed(1)},` +
        `${pressure},` +
        `${deviceTemp},` +
        `${filter.getOutsideTemperatureCelsius().toFixed(2)},` +
        `${filter.getDeviceTemperatureBias().toFixed(2)},` +
        `${filter.getTemperatureRate().toFixed(4)},` +
        `${filter.getTemperatureUncertainty().toFixed(3)}`
      );
    }
  }

  console.log('\nFilter processing complete!');
  console.log(filter.toString());
}

// Run the main function
if (require.main === module) {
  main();
}

export { main };
