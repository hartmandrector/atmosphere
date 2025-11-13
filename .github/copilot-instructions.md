# Atmosphere Project - Copilot Instructions

## Project Overview
TypeScript project for testing a temperature Kalman filter implementation and sensor fusion for VR head-mounted device alignment. Processes GPS and sensor data from FlySight 2 skydiving computer.

## Project Structure
- `src/` - TypeScript source files
  - `filters/` - Kalman filter implementations
    - `FilterTemp.ts` - Basic 3D filter (outside temp, bias, rate)
    - `FilterTempPhysics.ts` - Physics-based 5D filter with thermal dynamics
  - `parsers/` - CSV data parsers for TRACK.CSV and SENSOR.CSV
  - `utils/` - Utility classes (tensor operations, heading calculation, sensor fusion)
- `public/data/` - Default CSV data files
- `dist/` - Compiled JavaScript output

## Development Guidelines
- Use TypeScript strict mode
- Implement tensor operations for matrix calculations
- Parse CSV data from GPS (TRACK.CSV) and sensor (SENSOR.CSV) files
- Generate plottable output from temperature filter
- Calculate magnetic and GPS headings for VR alignment

## Architecture

### Core Components

- **src/main.ts** - Main application entry point handling file upload, drag/drop UI, and processing pipeline orchestration
- **src/plotter.ts** - Data processing pipeline with Kalman filtering and canvas rendering with zoom/pan
- **src/utils/SensorFusion.ts** - Sensor fusion algorithm for IMU alignment between FlySight and Quest headset

### Key Classes and Functions

#### Temperature Filters

Two Kalman filter implementations with different approaches:

**FilterTemp (Basic 3D)**
- State: `[outside_temp, device_bias, temp_rate]`
- Assumes constant device bias
- Simple barometric inversion + bias learning
- Good for initial prototyping

**FilterTempPhysics (Physics-Based 5D)**
- State: `[outside_temp, device_temp, cooling_coeff_α, heating_rate_β, temp_rate]`
- Models heat transfer physics: `dT_device/dt = -α(v) × (T_device - T_outside) + β`
- Cooling coefficient α adjusts with airspeed: `α(v) = α_base × (1 + sqrt(v))`
- Explicitly models thermal time constant and convective cooling
- More accurate during airspeed changes (freefall vs canopy)

Physics model details:
- Internal heating from GPS/Bluetooth → constant β (K/s)
- Convective cooling → α × ΔT, where α increases with airspeed
- Thermal mass → first-order lag in device temperature response
- Airspeed factor: At v=0 (natural convection), α minimal; at v=50 m/s (freefall), α increases ~8×

#### Sensor Fusion Algorithm

The sensor fusion algorithm calculates the rotation matrix and heading offset between two IMU systems (FlySight and Meta Quest):

1. **Acceleration Correlation**: Uses low-pass filtered acceleration signals to find optimal rotation by maximizing correlation
2. **Rotation Search**: Iterates through Euler angles (yaw, pitch, roll) to find best alignment
   - Yaw: Full 360° search (primary rotation for head-mounted devices)
   - Pitch/Roll: ±15° search (small deviations expected)
3. **Magnetometer Alignment**: After rotation, uses tilt-compensated magnetic heading to calculate heading offset
4. **Quality Metrics**: Provides correlation coefficient and confidence score

Key functions:
- `alignSensors()` - Main entry point, returns AlignmentResult with rotation matrix and metrics
- `findRotationMatrix()` - Searches rotation space for best acceleration correlation
- `calculateHeadingOffset()` - Computes magnetic heading offset in Quest frame
- `Matrix3x3.fromEuler()` - Creates rotation matrix from Euler angles (ZYX order)

Usage:
```typescript
const result = alignSensors(flysightSamples, questSamples, questHeading);
// result.rotationMatrix - 3x3 rotation matrix (FlySight → Quest)
// result.yaw, pitch, roll - Euler angles in degrees
// result.headingOffset - Magnetic heading offset in degrees
// result.correlation - Signal correlation (0-1)
// result.confidence - Overall confidence (0-1)
```

### Data Processing Pipeline

1. User drags FlySight folder with TRACK.CSV and SENSOR.CSV
2. CSV parser extracts GPS coordinates, timestamps, altitude, and sensor data
3. **Temperature Mode**: Kalman filter estimates outside temperature with device bias
4. **Heading Mode**: Calculates magnetic heading (tilt-compensated) and GPS heading
5. **Sensor Fusion Mode**: User loads Quest IMU data, algorithm calculates rotation offset
6. Interactive canvas displays data with zoom/pan controls

## Project Status
- [x] Create .github/copilot-instructions.md
- [x] Get project setup information
- [x] Scaffold TypeScript project structure
- [x] Copy CSV data to public folder
- [x] Convert FilterTemp.java to TypeScript
- [x] Install dependencies and compile
- [x] Create build/run tasks
- [x] Implement heading calculation (magnetic + GPS)
- [x] Create dual-mode visualization (Temperature/Heading)
- [x] Implement sensor fusion algorithm
- [x] Add sensor fusion UI panel
- [x] Finalize documentation

## Usage
```powershell
npm install       # Install dependencies
npm run build     # Compile TypeScript
npm start         # Run the temperature filter
npm run dev       # Build and run in one step
```

## Output Format
The program outputs CSV data showing:
- Time (seconds since Unix epoch)
- Barometric pressure (hPa)
- Device temperature (°C)
- Filtered outside temperature (°C)
- Device temperature bias (K)
- Temperature rate of change (K/s)
- Filter uncertainty (K)

## Sensor Fusion Workflow

1. Load FlySight data (drag folder with TRACK.CSV and SENSOR.CSV)
2. Switch to "Heading" view mode
3. Click "Load Quest IMU Data" and select CSV file with format: `time,ax,ay,az`
4. Click "Run Sensor Fusion" to calculate alignment
5. Review results:
   - Yaw, Pitch, Roll (Euler angles in degrees)
   - Heading Offset (magnetic heading difference)
   - Correlation (quality of signal match)
   - Confidence (overall reliability)
   - Rotation Matrix (3x3 transformation from FlySight to Quest frame)
6. Click "Copy Results to Clipboard" to export for VR application

## VR Integration

The calculated rotation matrix and heading offset can be used to align FlySight compass data with Meta Quest headset orientation in VR skydiving applications. This enables automatic calibration of the virtual environment's north direction based on real-world magnetic north.
