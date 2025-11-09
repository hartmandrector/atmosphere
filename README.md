# Atmosphere - Temperature Kalman Filter

A TypeScript implementation of a temperature Kalman filter for estimating outside air temperature from GPS and sensor data.

## Project Overview

This project processes GPS track data and device sensor readings to estimate the actual outside air temperature. The filter combines:
- **Atmospheric model inversion**: Uses pressure and altitude to calculate temperature
- **Device bias learning**: Learns the offset between device sensors and outside air

## Project Structure

```
atmosphere/
├── src/
│   ├── filters/
│   │   ├── Filter.ts          # Base filter interface
│   │   └── FilterTemp.ts      # Temperature Kalman filter
│   ├── parsers/
│   │   ├── TrackParser.ts     # GPS data parser (TRACK.CSV)
│   │   └── SensorParser.ts    # Sensor data parser (SENSOR.CSV)
│   ├── utils/
│   │   └── Tensor.ts          # Matrix/tensor operations
│   └── index.ts               # Main entry point
├── public/data/
│   ├── TRACK.CSV              # GPS track data
│   └── SENSOR.CSV             # Sensor readings
├── dist/                      # Compiled JavaScript output
├── package.json
├── tsconfig.json
└── README.md
```

## Data Format

### TRACK.CSV (GPS Data)
- `millis`: Timestamp in milliseconds
- `latitude`, `longitude`: Position
- `altitude_gps`: GPS altitude in meters
- `pressure_raw`: Atmospheric pressure in Pa
- `numSV`: Number of satellites

### SENSOR.CSV (Sensor Data)
- `millis`: Timestamp in milliseconds
- `temp`: Device temperature in Kelvin
- `pressure`: Atmospheric pressure in Pa
- `humidity`: Relative humidity
- Accelerometer, gyroscope, magnetometer readings

## Installation

```powershell
npm install
```

## Build

```powershell
npm run build
```

## Run

```powershell
npm start
```

Or for development (build + run):

```powershell
npm run dev
```

## Output

The program outputs CSV data to console:
```
Time(s),GPS_Temp(C),Device_Temp(C),Filtered_Temp(C),Bias(K),Rate(K/s),Uncertainty(K)
```

This data can be plotted to visualize:
- Temperature estimates from GPS/pressure vs device sensors
- Filtered outside air temperature
- Device temperature bias over time
- Temperature rate of change
- Filter uncertainty

## Filter Details

The temperature Kalman filter maintains a 3D state vector:
- **[0]** outside_air_temperature (K)
- **[1]** device_temperature_bias (K) - offset between device and outside
- **[2]** temperature_rate_of_change (K/s)

### Physical Constants
- Uses ISA standard atmosphere model
- Barometric formula for pressure-altitude-temperature relationship
- Standard lapse rate: 0.0065 K/m

### Filter Parameters
- Pressure measurement variance: 100 Pa²
- Device temperature variance: 4 K²
- Temperature process variance: 0.01 K²/s²
- Bias process variance: 0.001 K²/s²
- Rate process variance: 0.1 (K/s)²/s²

## License

MIT
