# Atmosphere Project - Copilot Instructions

## Project Overview
TypeScript project for testing a temperature Kalman filter implementation that processes GPS and sensor data.

## Project Structure
- `src/` - TypeScript source files
  - `filters/` - Kalman filter implementations (FilterTemp)
  - `parsers/` - CSV data parsers for TRACK.CSV and SENSOR.CSV
  - `utils/` - Utility classes (tensor operations)
- `public/data/` - Default CSV data files
- `dist/` - Compiled JavaScript output

## Development Guidelines
- Use TypeScript strict mode
- Implement tensor operations for matrix calculations
- Parse CSV data from GPS (TRACK.CSV) and sensor (SENSOR.CSV) files
- Generate plottable output from temperature filter

## Architecture

### Core Components

- **src/main.ts** - Main application entry point handling file upload, drag/drop UI, and processing pipeline orchestration
- **src/plotter.ts** - data processing pipeline with Kalman filtering,  and canvas rendering with zoom/pan


### Key Classes and Functions

-
- `parseCSV()` - 
- `plotData()` - GPS processing pipeline with Kalman filtering and 60Hz interpolation

### Data Processing Pipeline


2. CSV parser extracts GPS coordinates, timestamps, and altitude from TRACK and SENSOR files
6. Both filtered temperature points (blue, 4px radius) and interpolated points (red, 1px radius) are rendered on canvas
7. Interactive zoom/pan controls allow detailed temerature inspection


## Project Status
- [x] Create .github/copilot-instructions.md
- [x] Get project setup information
- [x] Scaffold TypeScript project structure
- [x] Copy CSV data to public folder
- [x] Convert FilterTemp.java to TypeScript
- [x] Install dependencies and compile
- [x] Create build/run tasks
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
