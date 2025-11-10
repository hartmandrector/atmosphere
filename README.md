# Atmosphere - Temperature Kalman Filter

A browser-based temperature Kalman filter for estimating outside air temperature from FlySight 2 GPS and sensor data.

## Project Overview

This project provides an interactive web application that processes GPS track data and device sensor readings to estimate the actual outside air temperature. The filter combines:
- **Atmospheric model inversion**: Uses pressure and altitude to calculate temperature via barometric formula
- **Device bias learning**: Learns the offset between device sensors and outside air
- **Real-time visualization**: Interactive plotting with zoom, pan, and parameter tuning

## Features

- 🌡️ **Temperature Estimation**: Combines pressure/altitude with device temperature to estimate outside air temp
- 📊 **Interactive Plotting**: Multi-axis visualization with zoom and pan
- 🎛️ **Parameter Tuning**: Adjust Kalman filter parameters in real-time
- 📁 **Drag & Drop**: Load new FlySight 2 data folders directly in browser
- 🔄 **Sensor Selection**: Choose between BARO, HUM, MAG, or IMU temperature sensors
- 🌍 **ISA Reference**: Display ISA standard atmosphere temperature with adjustable offset

## Project Structure

```
atmosphere/
├── src/
│   ├── filters/
│   │   ├── Filter.ts          # Base filter interface
│   │   └── FilterTemp.ts      # Temperature Kalman filter (3D state)
│   ├── parsers/
│   │   ├── TrackParser.ts     # GPS data parser (TRACK.CSV)
│   │   └── SensorParser.ts    # Sensor data parser (SENSOR.CSV)
│   ├── utils/
│   │   └── Tensor.ts          # Matrix/tensor operations
│   ├── main.ts                # Browser app with plotting
│   └── index.ts               # CLI entry point
├── public/data/
│   ├── TRACK.CSV              # Sample GPS track data
│   └── SENSOR.CSV             # Sample sensor readings
├── index.html                 # Browser UI
├── package.json
├── tsconfig.json
└── README.md
```

## Installation

```powershell
npm install
```

## Usage

### Start Development Server

```powershell
npm run dev
```

This will start Vite development server at `http://localhost:5174/` (or another port if 5174 is in use).

### Load FlySight 2 Data

1. Open your browser to the localhost URL
2. **Drag and drop** a FlySight 2 data folder onto the browser window
   - The folder should contain `TRACK.CSV` and `SENSOR.CSV` files
   - Example: Drag the `15-06-28` folder from your FlySight 2 device
3. The app will parse and plot the data automatically

### Adjust Filter Parameters

Click the **"Filter Parameters"** button to show/hide the control panel:
- **Pressure Measurement Variance** (Pa²): Barometer accuracy
- **Device Temp Variance** (K²): Temperature sensor accuracy  
- **Temperature Process Variance** (K²/s²): Rate of natural temperature change
- **Bias Process Variance** (K²/s²): Rate of device bias change
- **Rate Process Variance** ((K/s)²/s²): Rate of temperature acceleration
- **Temperature Offset** (K): Deviation from ISA standard atmosphere

After adjusting parameters, click **"Apply & Reprocess"** to recalculate with new settings.

### Select Temperature Sensor

Use the **"Temp Source"** dropdown to choose which sensor provides device temperature:
- **BARO**: Barometer temperature sensor
- **HUM**: Humidity sensor temperature
- **MAG**: Magnetometer temperature
- **IMU**: IMU (accelerometer/gyroscope) temperature

The filter will use the same pressure data (from BARO) but different device temperature readings.

## Visualization

The plot shows:
- 🔴 **Red line**: Device temperature (from selected sensor)
- 🔵 **Blue line**: Filtered outside air temperature (Kalman filter estimate)
- 🟢 **Green line**: ISA standard atmosphere temperature (at GPS altitude)
- 🟢 **Bright green line**: ISA + temperature offset
- 🟡 **Yellow line**: Device temperature bias (right axis)
- 🟣 **Purple line**: GPS altitude (far right axis)

**Controls:**
- **Mouse drag**: Pan the view
- **Mouse wheel**: Zoom in/out
- **Reset View button**: Return to default zoom level

## Data Format

### FlySight 2 CSV Format

Both TRACK.CSV and SENSOR.CSV use a custom format with metadata headers:

```
$FLYS,<device_info>
$VAR,<variable_names>
$COL,<column_names>
$UNIT,<units>
$DATA
<data_rows>
```

#### TRACK.CSV (GPS Data)
- `$GNSS,time,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,numSV`
- GPS position, altitude, velocity, accuracy metrics

#### SENSOR.CSV (Sensor Readings)
- `$TIME,time,tow,week` - GPS time synchronization
- `$BARO,time,pressure,temperature` - Barometer readings
- `$HUM,time,humidity,temperature` - Humidity sensor
- `$MAG,time,x,y,z,temperature` - Magnetometer
- `$IMU,time,wx,wy,wz,ax,ay,az,temperature` - Inertial measurement unit

All timestamps are synchronized to GPS epoch (January 6, 1980) using `$TIME` entries.

## Filter Details

The temperature Kalman filter maintains a 3D state vector:
- **[0]** outside_air_temperature (K)
- **[1]** device_temperature_bias (K) - offset between device and outside
- **[2]** temperature_rate_of_change (K/s)

### Physical Constants
- Uses ISA standard atmosphere model (T₀=288.15K, P₀=101325Pa)
- Barometric formula inversion for pressure-altitude-temperature relationship
- Standard lapse rate: 0.0065 K/m
- Temperature offset accounts for non-standard atmospheric conditions

### Default Filter Parameters
- Pressure measurement variance: 100 Pa²
- Device temperature variance: 4 K²
- Temperature process variance: 0.01 K²/s²
- Bias process variance: 0.001 K²/s²
- Rate process variance: 0.1 (K/s)²/s²
- Temperature offset: 10 K (adjustable from -20 to +50 K)

## How It Works

1. **Pressure Measurements**: The barometric formula is inverted to estimate temperature from pressure and altitude
2. **Device Temperature**: The selected sensor provides device temperature readings
3. **Kalman Filter**: Fuses both measurements to estimate actual outside temperature while learning device bias
4. **Bias Learning**: Over time, the filter learns how much warmer/cooler the device is compared to outside air
5. **Real-time Adjustment**: Parameter sliders allow tuning the filter behavior for different conditions

## Building for Production

```powershell
npm run build
```

The compiled output will be in the `dist/` folder.

## CLI Mode

For command-line processing (outputs CSV to console):

```powershell
npm start
```

## License

MIT
