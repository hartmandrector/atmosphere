import { FilterTemp } from './filters/FilterTemp';
import { FilterTempPhysics } from './filters/FilterTempPhysics';
import { OrientationFilter } from './filters/OrientationFilter';
import { TrackParser, TrackPoint } from './parsers/TrackParser';
import { SensorParser, SensorData } from './parsers/SensorParser';
import { Renderer3D } from './Renderer3D';
import { PlaybackController } from './PlaybackController';
import { TemperatureEstimator, TemperatureEstimatorParams, TemperatureEstimateResult } from './utils/TemperatureEstimator';
import { OffsetVisualizer, OffsetVisualizerParams, OffsetVisualizerState } from './OffsetVisualizer';

export interface PlotPoint {
  time: number;
  deviceTemp?: number;  // in °F
  filteredTemp?: number;  // in °F
  standardTemp?: number;  // in °F - ISA standard atmosphere temperature
  standardTempOffset?: number;  // in °F - ISA standard + offset
  bias?: number;  // in K
  altitude?: number;  // in meters
  coolingCoeff?: number;  // α in 1/s (physics filter)
  heatingRate?: number;   // β in K/s (physics filter)
  roll?: number;  // in degrees (orientation)
  pitch?: number;  // in degrees (orientation)
  yaw?: number;  // in degrees (orientation)
}

// Helper function to convert Celsius to Fahrenheit
function celsiusToFahrenheit(celsius: number): number {
  return celsius * 9/5 + 32;
}

// Calculate ISA standard atmosphere temperature from altitude
// T = T₀ - LAPSE_RATE × altitude
function getStandardTemperature(altitude: number): number {
  const TEMP_0 = 288.15;  // ISA standard temperature at sea level (K) = 15°C
  const LAPSE_RATE = 0.0065;  // K/m
  const tempK = TEMP_0 - LAPSE_RATE * altitude;
  const tempC = tempK - 273.15;
  return celsiusToFahrenheit(tempC);
}

// Calculate ISA standard atmosphere temperature with offset from altitude
function getStandardTemperatureWithOffset(altitude: number, offsetK: number): number {
  const TEMP_0 = 288.15;  // ISA standard temperature at sea level (K) = 15°C
  const LAPSE_RATE = 0.0065;  // K/m
  const tempK = TEMP_0 - LAPSE_RATE * altitude + offsetK;
  const tempC = tempK - 273.15;
  return celsiusToFahrenheit(tempC);
}

class Plotter {
  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;
  private data: PlotPoint[] = [];
  private viewMode: 'temperature' | 'tempphysics' | 'orientation' = 'temperature';
  private minTime = 0;
  private maxTime = 0;
  private minTemp = 0;
  private maxTemp = 50;
  private minBias = 0;
  private maxBias = 1;
  private minAltitude = 0;
  private maxAltitude = 1000;
  private minAngle = -180;
  private maxAngle = 180;
  private minCooling = 0;
  private maxCooling = 0.2;
  private minHeating = 0;
  private maxHeating = 2;
  private offsetX = 0;
  private offsetY = 0;
  private scale = 1;
  private isDragging = false;
  private lastX = 0;
  private lastY = 0;
  private cursorTime: number | null = null;  // Current playback time for cursor

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d')!;
    this.resizeCanvas();
    window.addEventListener('resize', () => this.resizeCanvas());
    
    // Mouse controls for pan/zoom
    this.canvas.addEventListener('mousedown', (e) => this.onMouseDown(e));
    this.canvas.addEventListener('mousemove', (e) => this.onMouseMove(e));
    this.canvas.addEventListener('mouseup', () => this.onMouseUp());
    this.canvas.addEventListener('mouseleave', () => this.onMouseUp());
    this.canvas.addEventListener('wheel', (e) => this.onWheel(e));
  }

  private resizeCanvas() {
    this.canvas.width = this.canvas.clientWidth;
    this.canvas.height = this.canvas.clientHeight;
    this.render();
  }

  private onMouseDown(e: MouseEvent) {
    this.isDragging = true;
    this.lastX = e.clientX;
    this.lastY = e.clientY;
  }

  private onMouseMove(e: MouseEvent) {
    if (!this.isDragging) return;
    const dx = e.clientX - this.lastX;
    const dy = e.clientY - this.lastY;
    this.offsetX += dx;
    this.offsetY += dy;
    this.lastX = e.clientX;
    this.lastY = e.clientY;
    this.render();
  }

  private onMouseUp() {
    this.isDragging = false;
  }

  private onWheel(e: WheelEvent) {
    e.preventDefault();
    const delta = e.deltaY > 0 ? 0.9 : 1.1;
    this.scale *= delta;
    this.scale = Math.max(0.1, Math.min(10, this.scale));
    this.render();
  }

  resetView() {
    this.offsetX = 0;
    this.offsetY = 0;
    this.scale = 1;
    this.render();
  }
  
  setCursor(time: number) {
    this.cursorTime = time;
    this.render();
  }
  
  clearCursor() {
    this.cursorTime = null;
    this.render();
  }

  setViewMode(mode: 'temperature' | 'tempphysics' | 'orientation') {
    this.viewMode = mode;
    this.render();
  }

  setData(data: PlotPoint[]) {
    this.data = data;
    
    if (data.length > 0) {
      this.minTime = data[0].time;
      this.maxTime = data[data.length - 1].time;
      
      // Calculate temperature range
      let minT = Infinity;
      let maxT = -Infinity;
      let minB = Infinity;
      let maxB = -Infinity;
      let minA = Infinity;
      let maxA = -Infinity;
      let minC = Infinity;
      let maxC = -Infinity;
      let minH = Infinity;
      let maxH = -Infinity;
      let minAngle = Infinity;
      let maxAngle = -Infinity;
      for (const pt of data) {
        if (pt.deviceTemp !== undefined) {
          minT = Math.min(minT, pt.deviceTemp);
          maxT = Math.max(maxT, pt.deviceTemp);
        }
        if (pt.filteredTemp !== undefined) {
          minT = Math.min(minT, pt.filteredTemp);
          maxT = Math.max(maxT, pt.filteredTemp);
        }
        if (pt.standardTemp !== undefined) {
          minT = Math.min(minT, pt.standardTemp);
          maxT = Math.max(maxT, pt.standardTemp);
        }
        if (pt.standardTempOffset !== undefined) {
          minT = Math.min(minT, pt.standardTempOffset);
          maxT = Math.max(maxT, pt.standardTempOffset);
        }
        if (pt.bias !== undefined) {
          minB = Math.min(minB, pt.bias);
          maxB = Math.max(maxB, pt.bias);
        }
        if (pt.altitude !== undefined) {
          minA = Math.min(minA, pt.altitude);
          maxA = Math.max(maxA, pt.altitude);
        }
        if (pt.coolingCoeff !== undefined) {
          minC = Math.min(minC, pt.coolingCoeff);
          maxC = Math.max(maxC, pt.coolingCoeff);
        }
        if (pt.heatingRate !== undefined) {
          minH = Math.min(minH, pt.heatingRate);
          maxH = Math.max(maxH, pt.heatingRate);
        }
        // Track angle ranges for orientation mode
        if (pt.roll !== undefined) {
          minAngle = Math.min(minAngle, pt.roll);
          maxAngle = Math.max(maxAngle, pt.roll);
        }
        if (pt.pitch !== undefined) {
          minAngle = Math.min(minAngle, pt.pitch);
          maxAngle = Math.max(maxAngle, pt.pitch);
        }
        if (pt.yaw !== undefined) {
          minAngle = Math.min(minAngle, pt.yaw);
          maxAngle = Math.max(maxAngle, pt.yaw);
        }
      }
      this.minTemp = Math.floor(minT - 2);
      this.maxTemp = Math.ceil(maxT + 2);
      this.minBias = Math.floor(minB * 100) / 100 - 0.05;
      this.maxBias = Math.ceil(maxB * 100) / 100 + 0.05;
      this.minAltitude = Math.floor(minA / 100) * 100;
      this.maxAltitude = Math.ceil(maxA / 100) * 100;
      
      // Set cooling and heating ranges (with defaults if no data)
      if (minC !== Infinity && maxC !== -Infinity) {
        this.minCooling = Math.max(0, Math.floor(minC * 1000) / 1000 - 0.01);
        this.maxCooling = Math.ceil(maxC * 1000) / 1000 + 0.01;
      }
      if (minH !== Infinity && maxH !== -Infinity) {
        this.minHeating = Math.max(0, Math.floor(minH * 100) / 100 - 0.1);
        this.maxHeating = Math.ceil(maxH * 100) / 100 + 0.1;
      }
      
      // Set angle ranges for orientation mode (default to ±180° if no data)
      if (minAngle !== Infinity && maxAngle !== -Infinity) {
        const padding = (maxAngle - minAngle) * 0.1 || 10; // 10% padding or 10° minimum
        this.minAngle = Math.max(-180, Math.floor(minAngle - padding));
        this.maxAngle = Math.min(180, Math.ceil(maxAngle + padding));
      }
    }
    
    this.resetView();
  }

  private render() {
    const { width, height } = this.canvas;
    const ctx = this.ctx;
    
    // Clear
    ctx.fillStyle = '#1e1e1e';
    ctx.fillRect(0, 0, width, height);
    
    if (this.data.length === 0) return;
    
    // Margins
    const marginLeft = 60;
    const marginRight = this.viewMode === 'tempphysics' ? 220 : (this.viewMode === 'temperature' ? 170 : 70);
    const marginTop = 20;
    const marginBottom = 40;
    
    const plotWidth = width - marginLeft - marginRight;
    const plotHeight = height - marginTop - marginBottom;
    
    // Apply transform
    ctx.save();
    ctx.translate(this.offsetX, this.offsetY);
    ctx.scale(this.scale, this.scale);
    
    // Draw axes
    ctx.strokeStyle = '#3e3e42';
    ctx.lineWidth = 1 / this.scale;
    ctx.beginPath();
    ctx.moveTo(marginLeft, marginTop);
    ctx.lineTo(marginLeft, height - marginBottom);
    ctx.lineTo(width - marginRight, height - marginBottom);
    ctx.lineTo(width - marginRight, marginTop);
    ctx.stroke();
    
    // Draw grid and labels
    this.drawGrid(ctx, marginLeft, marginTop, plotWidth, plotHeight, width - marginRight, this.viewMode);
    
    if (this.viewMode === 'temperature') {
      // Plot temperature mode data
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'deviceTemp', '#ff6b6b', 2 / this.scale, 'temp');
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'filteredTemp', '#4dabf7', 2 / this.scale, 'temp');
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'standardTemp', '#51cf66', 2 / this.scale, 'temp');
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'standardTempOffset', '#22c55e', 2 / this.scale, 'temp');
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'bias', '#ffd43b', 2 / this.scale, 'bias');
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'altitude', '#a78bfa', 1.5 / this.scale, 'altitude');
    } else if (this.viewMode === 'tempphysics') {
      // Plot physics temperature mode data
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'deviceTemp', '#ff6b6b', 2 / this.scale, 'temp');
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'filteredTemp', '#4dabf7', 2 / this.scale, 'temp');
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'standardTemp', '#51cf66', 2 / this.scale, 'temp');
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'standardTempOffset', '#22c55e', 2 / this.scale, 'temp');
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'coolingCoeff', '#ffd43b', 2 / this.scale, 'cooling');
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'heatingRate', '#ff8c42', 2 / this.scale, 'heating');
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'altitude', '#a78bfa', 1.5 / this.scale, 'altitude');
    } else if (this.viewMode === 'orientation') {
      // Plot orientation data (Euler angles)
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'roll', '#ff6b6b', 2 / this.scale, 'angle');
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'pitch', '#4dabf7', 2 / this.scale, 'angle');
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'yaw', '#51cf66', 2 / this.scale, 'angle');
      this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'altitude', '#a78bfa', 1.5 / this.scale, 'altitude');
    }
    
    // Draw playback cursor if active
    if (this.cursorTime !== null) {
      const timeRange = this.maxTime - this.minTime;
      const cursorX = marginLeft + ((this.cursorTime - this.minTime) / timeRange) * plotWidth;
      
      ctx.strokeStyle = '#ffd43b';
      ctx.lineWidth = 2 / this.scale;
      ctx.beginPath();
      ctx.moveTo(cursorX, marginTop);
      ctx.lineTo(cursorX, marginTop + plotHeight);
      ctx.stroke();
    }
    
    ctx.restore();
  }

  private drawGrid(ctx: CanvasRenderingContext2D, x: number, y: number, 
                   width: number, height: number, rightAxisX: number, mode: 'temperature' | 'tempphysics' | 'orientation') {
    const timeRange = this.maxTime - this.minTime;
    const tempRange = this.maxTemp - this.minTemp;
    const biasRange = this.maxBias - this.minBias;
    const altitudeRange = this.maxAltitude - this.minAltitude;
    const angleRange = this.maxAngle - this.minAngle;
    const coolingRange = this.maxCooling - this.minCooling;
    const heatingRange = this.maxHeating - this.minHeating;
    
    // Vertical grid (time)
    ctx.strokeStyle = '#2d2d30';
    ctx.fillStyle = '#858585';
    ctx.font = `${10 / this.scale}px monospace`;
    ctx.lineWidth = 1 / this.scale;
    
    const timeStep = this.getStep(timeRange / 10);
    for (let t = Math.ceil(this.minTime / timeStep) * timeStep; t <= this.maxTime; t += timeStep) {
      const px = x + ((t - this.minTime) / timeRange) * width;
      ctx.beginPath();
      ctx.moveTo(px, y);
      ctx.lineTo(px, y + height);
      ctx.stroke();
      
      const timeLabel = this.formatTime(t);
      ctx.fillText(timeLabel, px - 20 / this.scale, y + height + 20 / this.scale);
    }
    
    if (mode === 'temperature') {
      // Temperature mode axes
      // Horizontal grid (temperature - left axis)
      const tempStep = this.getStep(tempRange / 8);
      for (let temp = Math.ceil(this.minTemp / tempStep) * tempStep; temp <= this.maxTemp; temp += tempStep) {
        const py = y + height - ((temp - this.minTemp) / tempRange) * height;
        ctx.beginPath();
        ctx.moveTo(x, py);
        ctx.lineTo(x + width, py);
        ctx.stroke();
        
        ctx.fillStyle = '#858585';
        ctx.fillText(`${temp}°F`, x - 45 / this.scale, py + 3 / this.scale);
      }
      
      // Right axis labels (bias in K)
      ctx.fillStyle = '#ffd43b';
      const biasStep = this.getStep(biasRange / 8);
      for (let bias = Math.ceil(this.minBias / biasStep) * biasStep; bias <= this.maxBias; bias += biasStep) {
        const py = y + height - ((bias - this.minBias) / biasRange) * height;
        const label = `${bias.toFixed(2)}K`;
        ctx.fillText(label, rightAxisX + 5 / this.scale, py + 3 / this.scale);
      }
      
      // Far right axis labels (altitude in m)
      ctx.fillStyle = '#a78bfa';
      const altStep = this.getStep(altitudeRange / 8);
      for (let alt = Math.ceil(this.minAltitude / altStep) * altStep; alt <= this.maxAltitude; alt += altStep) {
        const py = y + height - ((alt - this.minAltitude) / altitudeRange) * height;
        const label = `${alt}m`;
        ctx.fillText(label, rightAxisX + 50 / this.scale, py + 3 / this.scale);
      }
    } else if (mode === 'tempphysics') {
      // TempPhysics mode axes
      // Horizontal grid (temperature - left axis)
      const tempStep = this.getStep(tempRange / 8);
      for (let temp = Math.ceil(this.minTemp / tempStep) * tempStep; temp <= this.maxTemp; temp += tempStep) {
        const py = y + height - ((temp - this.minTemp) / tempRange) * height;
        ctx.beginPath();
        ctx.moveTo(x, py);
        ctx.lineTo(x + width, py);
        ctx.stroke();
        
        ctx.fillStyle = '#858585';
        ctx.fillText(`${temp}°F`, x - 45 / this.scale, py + 3 / this.scale);
      }
      
      // Right axis 1: Cooling coefficient (1/s)
      ctx.fillStyle = '#ffd43b';
      const coolingStep = this.getStep(coolingRange / 8);
      for (let cool = Math.ceil(this.minCooling / coolingStep) * coolingStep; cool <= this.maxCooling; cool += coolingStep) {
        const py = y + height - ((cool - this.minCooling) / coolingRange) * height;
        const label = `${cool.toFixed(3)}`;
        ctx.fillText(label, rightAxisX + 5 / this.scale, py + 3 / this.scale);
      }
      
      // Right axis 2: Heating rate (K/s)
      ctx.fillStyle = '#ff8c42';
      const heatingStep = this.getStep(heatingRange / 8);
      for (let heat = Math.ceil(this.minHeating / heatingStep) * heatingStep; heat <= this.maxHeating; heat += heatingStep) {
        const py = y + height - ((heat - this.minHeating) / heatingRange) * height;
        const label = `${heat.toFixed(2)}`;
        ctx.fillText(label, rightAxisX + 50 / this.scale, py + 3 / this.scale);
      }
      
      // Right axis 3: Altitude (m)
      ctx.fillStyle = '#a78bfa';
      const altStep2 = this.getStep(altitudeRange / 8);
      for (let alt = Math.ceil(this.minAltitude / altStep2) * altStep2; alt <= this.maxAltitude; alt += altStep2) {
        const py = y + height - ((alt - this.minAltitude) / altitudeRange) * height;
        const label = `${alt}m`;
        ctx.fillText(label, rightAxisX + 100 / this.scale, py + 3 / this.scale);
      }
    } else if (mode === 'orientation') {
      // Orientation mode axes
      // Horizontal grid (angles - left axis)
      ctx.fillStyle = '#858585';
      const angleStep = 45;
      for (let ang = -180; ang <= 180; ang += angleStep) {
        const py = y + height - ((ang - this.minAngle) / angleRange) * height;
        ctx.beginPath();
        ctx.moveTo(x, py);
        ctx.lineTo(x + width, py);
        ctx.stroke();
        
        const label = `${ang}°`;
        ctx.fillText(label, x - 40 / this.scale, py + 3 / this.scale);
      }
      
      // Right axis labels (altitude in m)
      ctx.fillStyle = '#a78bfa';
      const altStepOrient = this.getStep(altitudeRange / 8);
      for (let alt = Math.ceil(this.minAltitude / altStepOrient) * altStepOrient; alt <= this.maxAltitude; alt += altStepOrient) {
        const py = y + height - ((alt - this.minAltitude) / altitudeRange) * height;
        const label = `${alt}m`;
        ctx.fillText(label, rightAxisX + 5 / this.scale, py + 3 / this.scale);
      }
    }
  }

  private getStep(range: number): number {
    const magnitude = Math.pow(10, Math.floor(Math.log10(range)));
    const normalized = range / magnitude;
    if (normalized < 2) return magnitude * 0.5;
    if (normalized < 5) return magnitude;
    return magnitude * 2;
  }

  private formatTime(seconds: number): string {
    const date = new Date(seconds * 1000);
    return date.toISOString().substr(11, 8);
  }

  private plotLine(ctx: CanvasRenderingContext2D, x: number, y: number, 
                   width: number, height: number, field: keyof PlotPoint, 
                   color: string, lineWidth: number, axis: 'temp' | 'bias' | 'altitude' | 'cooling' | 'heating' | 'angle') {
    const timeRange = this.maxTime - this.minTime;
    let valueRange: number;
    let minValue: number;
    
    if (axis === 'bias') {
      valueRange = this.maxBias - this.minBias;
      minValue = this.minBias;
    } else if (axis === 'altitude') {
      valueRange = this.maxAltitude - this.minAltitude;
      minValue = this.minAltitude;
    } else if (axis === 'cooling') {
      valueRange = this.maxCooling - this.minCooling;
      minValue = this.minCooling;
    } else if (axis === 'heating') {
      valueRange = this.maxHeating - this.minHeating;
      minValue = this.minHeating;
    } else if (axis === 'angle') {
      valueRange = this.maxAngle - this.minAngle;
      minValue = this.minAngle;
    } else {
      valueRange = this.maxTemp - this.minTemp;
      minValue = this.minTemp;
    }
    
    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
    ctx.beginPath();
    
    let started = false;
    for (const pt of this.data) {
      const value = pt[field] as number | undefined;
      if (value === undefined) continue;
      
      const px = x + ((pt.time - this.minTime) / timeRange) * width;
      const py = y + height - ((value - minValue) / valueRange) * height;
      
      if (!started) {
        ctx.moveTo(px, py);
        started = true;
      } else {
        ctx.lineTo(px, py);
      }
    }
    
    ctx.stroke();
  }
}

class App {
  private plotter: Plotter;
  private dropZone: HTMLElement;
  private dropTarget: HTMLElement;
  private dataInfo: HTMLElement;
  private status: HTMLElement;
  private resetBtn: HTMLButtonElement;
  private toggleControlsBtn: HTMLButtonElement;
  private filterControls: HTMLElement;
  private applyParamsBtn: HTMLButtonElement;
  private resetParamsBtn: HTMLButtonElement;
  
  // Filter parameters
  private filterParams = {
    pressureVar: 100.0,
    deviceTempVar: 4.0,
    tempProcessVar: 0.01,
    biasProcessVar: 0.001,
    rateProcessVar: 0.1,
    tempOffset: 10.0
  };
  
  // Physics filter parameters
  private physicsFilterParams = {
    pressureVar: 100.0,
    deviceTempVar: 4.0,
    tempProcessVar: 0.01,
    biasProcessVar: 0.001,   // Device bias process noise
    coolingProcessVar: 0.01,  // α process noise
    rateProcessVar: 0.1,
    heatingRate: 0.5  // β parameter (K/s)
  };
  
  // Orientation filter parameters
  private orientationFilterParams = {
    gyroNoiseVar: 0.005,
    gyroBiasNoiseVar: 0.0001,
    accelNoiseVar: 1.0,
    magNoiseVar: 1.0
  };
  
  // Temperature source selection
  private tempSource: 'BARO' | 'HUM' | 'MAG' | 'IMU' = 'BARO';
  
  // Store loaded data for reprocessing
  private trackText: string = '';
  private sensorText: string = '';
  
  // Temperature Estimator for GPS-based offset calculation
  private temperatureEstimator: TemperatureEstimator = new TemperatureEstimator();
  private currentGpsInfo: { latitude: number; longitude: number; millis: number; altitude: number } | null = null;
  private lastEstimateResult: TemperatureEstimateResult | null = null;
  
  // 3D view and playback for orientation mode
  private renderer3D: Renderer3D | null = null;
  private playbackController: PlaybackController | null = null;
  private orientationPlotter: Plotter | null = null;
  
  // Offset visualizer for temperature offset charts
  private offsetVisualizer: OffsetVisualizer | null = null;

  constructor() {
    const canvas = document.getElementById('canvas') as HTMLCanvasElement;
    this.plotter = new Plotter(canvas);
    
    this.dropZone = document.getElementById('drop-zone')!;
    this.dropTarget = document.getElementById('drop-target')!;;
    this.dataInfo = document.getElementById('data-info')!;
    this.status = document.getElementById('status')!;
    this.resetBtn = document.getElementById('reset-btn') as HTMLButtonElement;
    this.toggleControlsBtn = document.getElementById('toggle-controls-btn') as HTMLButtonElement;
    this.filterControls = document.getElementById('filter-controls')!;
    this.applyParamsBtn = document.getElementById('apply-params-btn') as HTMLButtonElement;
    this.resetParamsBtn = document.getElementById('reset-params-btn') as HTMLButtonElement;
    
    this.setupDragDrop();
    this.setupFilterControls();
    this.setupPhysicsFilterControls();
    this.setupOrientationFilterControls();
    this.setupTempSourceSelector();
    this.setupViewModeSelector();
    this.setupTemperatureEstimatorControls();
    this.setupOffsetVisualizerControls();
    this.resetBtn.addEventListener('click', () => this.plotter.resetView());
    
    // Load default data
    this.loadDefaultData();
  }

  private setupViewModeSelector() {
    const viewModeSelect = document.getElementById('view-mode') as HTMLSelectElement;
    const tempSourceLabel = document.getElementById('temp-source-label')!;
    const tempSourceSelect = document.getElementById('temp-source')!;
    const toggleControlsBtn = this.toggleControlsBtn;
    const legendTemperature = document.getElementById('legend-temperature')!;
    const legendTempPhysics = document.getElementById('legend-tempphysics')!;
    const legendOrientation = document.getElementById('legend-orientation')!;
    const filterControls = document.getElementById('filter-controls')!;
    const physicsFilterControls = document.getElementById('physics-filter-controls')!;
    const offsetFilterControls = document.getElementById('offset-filter-controls')!;
    
    viewModeSelect.addEventListener('change', () => {
      const mode = viewModeSelect.value as 'temperature' | 'tempphysics' | 'orientation' | 'offset';
      
      // Show/hide display areas
      const canvas = document.getElementById('canvas') as HTMLCanvasElement;
      const orientationDisplay = document.getElementById('orientation-display')!;
      const offsetDisplay = document.getElementById('offset-display')!;
      
      // Hide all displays first
      canvas.style.display = 'none';
      orientationDisplay.style.display = 'none';
      offsetDisplay.style.display = 'none';
      
      // Hide all legends
      legendTemperature.style.display = 'none';
      legendTempPhysics.style.display = 'none';
      legendOrientation.style.display = 'none';
      
      // Hide all control panels
      filterControls.style.display = 'none';
      physicsFilterControls.style.display = 'none';
      document.getElementById('orientation-filter-controls')!.style.display = 'none';
      offsetFilterControls.style.display = 'none';
      
      if (mode === 'offset') {
        offsetDisplay.style.display = 'flex';
        tempSourceLabel.style.display = 'none';
        tempSourceSelect.style.display = 'none';
        toggleControlsBtn.style.display = '';
        
        // Initialize offset visualizer if needed
        if (!this.offsetVisualizer) {
          const yearCanvas = document.getElementById('offset-year-canvas') as HTMLCanvasElement;
          const monthCanvas = document.getElementById('offset-month-canvas') as HTMLCanvasElement;
          const dayCanvas = document.getElementById('offset-day-canvas') as HTMLCanvasElement;
          
          this.offsetVisualizer = new OffsetVisualizer(yearCanvas, monthCanvas, dayCanvas);
          
          // If we have GPS data, initialize with it
          if (this.currentGpsInfo) {
            this.offsetVisualizer.setParams({
              latitude: this.currentGpsInfo.latitude,
              longitude: this.currentGpsInfo.longitude,
              altitude: this.currentGpsInfo.altitude
            });
          }
        }
        
        // Render the charts
        this.offsetVisualizer.render();
        
      } else if (mode === 'orientation') {
        orientationDisplay.style.display = 'flex';
        tempSourceLabel.style.display = 'none';
        tempSourceSelect.style.display = 'none';
        toggleControlsBtn.style.display = '';
        legendOrientation.style.display = 'flex';
        
        // Initialize 3D renderer if needed
        if (!this.renderer3D) {
          const viewer3D = document.getElementById('viewer-3d')!;
          this.renderer3D = new Renderer3D(viewer3D);
          
          // Initialize orientation canvas plotter
          const canvasOrientation = document.getElementById('canvas-orientation') as HTMLCanvasElement;
          this.orientationPlotter = new Plotter(canvasOrientation);
          this.orientationPlotter.setViewMode('orientation');
          
          const playBtn = document.getElementById('play-btn') as HTMLButtonElement;
          const pauseBtn = document.getElementById('pause-btn') as HTMLButtonElement;
          const stopBtn = document.getElementById('stop-btn') as HTMLButtonElement;
          const speedSlider = document.getElementById('speed-slider') as HTMLInputElement;
          const speedValue = document.getElementById('speed-value') as HTMLSpanElement;
          const timeDisplay = document.getElementById('time-display') as HTMLSpanElement;
          
          this.playbackController = new PlaybackController(
            this.renderer3D, playBtn, pauseBtn, stopBtn,
            speedSlider, speedValue, timeDisplay
          );
          
          // Pass the orientation plotter to the playback controller for cursor
          this.playbackController.setPlotter(this.orientationPlotter);
        }
        
        // Reprocess data when switching modes
        if (this.trackText && this.sensorText) {
          this.processData(this.trackText, this.sensorText);
        }
        
      } else {
        // Temperature or TempPhysics mode
        canvas.style.display = 'block';
        tempSourceLabel.style.display = '';
        tempSourceSelect.style.display = '';
        toggleControlsBtn.style.display = '';
        
        if (mode === 'temperature') {
          legendTemperature.style.display = 'flex';
          this.plotter.setViewMode('temperature');
        } else if (mode === 'tempphysics') {
          legendTempPhysics.style.display = 'flex';
          this.plotter.setViewMode('tempphysics');
        }
        
        // Reprocess data when switching modes
        if (this.trackText && this.sensorText) {
          this.processData(this.trackText, this.sensorText);
        }
      }
    });
  }

  private setupFilterControls() {
    // Note: Toggle controls visibility is handled in setupPhysicsFilterControls
    // to support both filter types based on view mode

    // Setup sliders
    const params: (keyof typeof this.filterParams)[] = [
      'pressureVar', 'deviceTempVar', 'tempProcessVar', 'biasProcessVar', 'rateProcessVar', 'tempOffset'
    ];

    params.forEach(param => {
      const slider = document.getElementById(param) as HTMLInputElement;
      const valueSpan = document.getElementById(`${param}-value`)!;
      
      slider.addEventListener('input', () => {
        const value = parseFloat(slider.value);
        this.filterParams[param] = value;
        if (param === 'tempOffset') {
          // Display both K and °F for temperature offset
          const degF = value * 1.8; // Temperature difference: 1 K = 1.8 °F
          valueSpan.textContent = `${value.toFixed(1)} K (${degF.toFixed(1)} °F)`;
        } else {
          valueSpan.textContent = value.toFixed(value < 1 ? 4 : value < 10 ? 1 : 0);
        }
      });
    });

    // Apply parameters and reprocess
    this.applyParamsBtn.addEventListener('click', () => {
      if (this.trackText && this.sensorText) {
        this.processData(this.trackText, this.sensorText);
      }
    });

    // Reset to defaults
    this.resetParamsBtn.addEventListener('click', () => {
      this.filterParams = {
        pressureVar: 100.0,
        deviceTempVar: 4.0,
        tempProcessVar: 0.01,
        biasProcessVar: 0.001,
        rateProcessVar: 0.1,
        tempOffset: 10.0
      };
      
      (document.getElementById('pressureVar') as HTMLInputElement).value = '100';
      (document.getElementById('deviceTempVar') as HTMLInputElement).value = '4.0';
      (document.getElementById('tempProcessVar') as HTMLInputElement).value = '0.01';
      (document.getElementById('biasProcessVar') as HTMLInputElement).value = '0.001';
      (document.getElementById('rateProcessVar') as HTMLInputElement).value = '0.1';
      (document.getElementById('tempOffset') as HTMLInputElement).value = '10.0';
      
      document.getElementById('pressureVar-value')!.textContent = '100';
      document.getElementById('deviceTempVar-value')!.textContent = '4.0';
      document.getElementById('tempProcessVar-value')!.textContent = '0.0100';
      document.getElementById('biasProcessVar-value')!.textContent = '0.0010';
      document.getElementById('rateProcessVar-value')!.textContent = '0.100';
      document.getElementById('tempOffset-value')!.textContent = '10.0 K (18.0 °F)';
      
      if (this.trackText && this.sensorText) {
        this.processData(this.trackText, this.sensorText);
      }
    });
  }

  private setupTemperatureEstimatorControls() {
    // Setup sliders for temperature estimator model parameters
    const estimatorParams: { id: string; key: keyof TemperatureEstimatorParams }[] = [
      { id: 'est-tEquator', key: 'T_EQUATOR' },
      { id: 'est-aLat', key: 'A_LAT' },
      { id: 'est-aYear', key: 'A_YEAR' },
      { id: 'est-aDay', key: 'A_DAY' },
      { id: 'est-h0', key: 'H0' },
      { id: 'est-alpha', key: 'ALPHA' },
      { id: 'est-beta', key: 'BETA' },
      { id: 'est-lapseRate', key: 'LAPSE_RATE' }
    ];

    estimatorParams.forEach(({ id, key }) => {
      const slider = document.getElementById(id) as HTMLInputElement;
      const valueSpan = document.getElementById(`${id}-value`)!;
      
      slider.addEventListener('input', () => {
        const value = parseFloat(slider.value);
        const currentParams = this.temperatureEstimator.getParams();
        currentParams[key] = value;
        this.temperatureEstimator.setParams(currentParams);
        valueSpan.textContent = value.toFixed(1);
        
        // Recalculate and update display if we have GPS data
        this.updateTemperatureEstimate();
      });
    });

    // Apply estimated offset button
    const applyEstimatedBtn = document.getElementById('apply-estimated-offset-btn') as HTMLButtonElement;
    applyEstimatedBtn.addEventListener('click', () => {
      if (this.lastEstimateResult) {
        // Apply the estimated offset to the filter's tempOffset parameter
        const offsetK = this.lastEstimateResult.offsetK;
        this.filterParams.tempOffset = offsetK;
        
        // Update the tempOffset slider and display
        const tempOffsetSlider = document.getElementById('tempOffset') as HTMLInputElement;
        const tempOffsetValue = document.getElementById('tempOffset-value')!;
        tempOffsetSlider.value = offsetK.toFixed(1);
        const degF = offsetK * 1.8;
        tempOffsetValue.textContent = `${offsetK.toFixed(1)} K (${degF.toFixed(1)} °F)`;
        
        // Reprocess data with new offset
        if (this.trackText && this.sensorText) {
          this.processData(this.trackText, this.sensorText);
        }
      }
    });

    // Reset estimator parameters to defaults
    const resetEstimatorBtn = document.getElementById('reset-estimator-params-btn') as HTMLButtonElement;
    resetEstimatorBtn.addEventListener('click', () => {
      const defaults: TemperatureEstimatorParams = {
        T_EQUATOR: 32.0,
        A_LAT: 23.0,
        A_YEAR: 16.0,
        A_DAY: 10.0,
        H0: 3.0,
        ALPHA: 1.2,
        BETA: 1.0,
        LAPSE_RATE: 6.5,
        SUMMER_PEAK_DAY: null  // auto-calculate from latitude
      };
      
      this.temperatureEstimator.setParams(defaults);
      
      // Update all sliders
      (document.getElementById('est-tEquator') as HTMLInputElement).value = '32.0';
      (document.getElementById('est-aLat') as HTMLInputElement).value = '23.0';
      (document.getElementById('est-aYear') as HTMLInputElement).value = '16.0';
      (document.getElementById('est-aDay') as HTMLInputElement).value = '10.0';
      (document.getElementById('est-h0') as HTMLInputElement).value = '3.0';
      (document.getElementById('est-alpha') as HTMLInputElement).value = '1.2';
      (document.getElementById('est-beta') as HTMLInputElement).value = '1.0';
      (document.getElementById('est-lapseRate') as HTMLInputElement).value = '6.5';
      
      document.getElementById('est-tEquator-value')!.textContent = '32.0';
      document.getElementById('est-aLat-value')!.textContent = '23.0';
      document.getElementById('est-aYear-value')!.textContent = '16.0';
      document.getElementById('est-aDay-value')!.textContent = '10.0';
      document.getElementById('est-h0-value')!.textContent = '3.0';
      document.getElementById('est-alpha-value')!.textContent = '1.2';
      document.getElementById('est-beta-value')!.textContent = '1.0';
      document.getElementById('est-lapseRate-value')!.textContent = '6.5';
      
      // Recalculate with defaults
      this.updateTemperatureEstimate();
    });
  }

  private setupOffsetVisualizerControls() {
    const monthNames = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec'];
    
    // Helper to update display info
    const updateOffsetDisplay = () => {
      if (!this.offsetVisualizer) return;
      
      const params = this.offsetVisualizer.getParams();
      const date = new Date(params.year, params.month - 1, params.day);
      
      document.getElementById('offset-location-info')!.textContent = 
        `Lat: ${params.latitude.toFixed(2)}° Lon: ${params.longitude.toFixed(2)}°`;
      document.getElementById('offset-date-info')!.textContent = 
        `Date: ${monthNames[params.month - 1]} ${params.day}, ${params.year}`;
      
      // Calculate and display offset for noon on selected day
      const estimator = new TemperatureEstimator(params.estimatorParams);
      date.setUTCHours(12, 0, 0, 0);
      const result = estimator.estimateTemperatureOffset(params.latitude, params.longitude, date.getTime());
      document.getElementById('offset-calc-info')!.textContent = 
        `Noon Offset: ${result.offsetK.toFixed(1)} K (${result.offsetF.toFixed(1)}°F)`;
    };
    
    // Location sliders
    const latSlider = document.getElementById('offset-lat') as HTMLInputElement;
    const lonSlider = document.getElementById('offset-lon') as HTMLInputElement;
    const altSlider = document.getElementById('offset-alt') as HTMLInputElement;
    
    latSlider.addEventListener('input', () => {
      const value = parseFloat(latSlider.value);
      document.getElementById('offset-lat-value')!.textContent = value.toFixed(2);
      if (this.offsetVisualizer) {
        this.offsetVisualizer.setParams({ latitude: value });
        updateOffsetDisplay();
      }
    });
    
    lonSlider.addEventListener('input', () => {
      const value = parseFloat(lonSlider.value);
      document.getElementById('offset-lon-value')!.textContent = value.toFixed(2);
      if (this.offsetVisualizer) {
        this.offsetVisualizer.setParams({ longitude: value });
        updateOffsetDisplay();
      }
    });
    
    altSlider.addEventListener('input', () => {
      const value = parseFloat(altSlider.value);
      document.getElementById('offset-alt-value')!.textContent = value.toFixed(0);
      if (this.offsetVisualizer) {
        this.offsetVisualizer.setParams({ altitude: value });
        updateOffsetDisplay();
      }
    });
    
    // Date sliders
    const dowSlider = document.getElementById('offset-dow') as HTMLInputElement;
    const weekSlider = document.getElementById('offset-week') as HTMLInputElement;
    const monthSlider = document.getElementById('offset-month') as HTMLInputElement;
    const daySlider = document.getElementById('offset-day') as HTMLInputElement;
    
    // Day of week adjusts the day within the current week
    dowSlider.addEventListener('input', () => {
      const dow = parseInt(dowSlider.value);
      document.getElementById('offset-dow-value')!.textContent = dow.toString();
      
      if (this.offsetVisualizer) {
        const params = this.offsetVisualizer.getParams();
        // Calculate the date for this day of week in current week
        const currentDate = new Date(params.year, params.month - 1, params.day);
        const currentDow = currentDate.getDay() || 7; // Convert Sunday=0 to 7
        const diff = dow - currentDow;
        currentDate.setDate(currentDate.getDate() + diff);
        
        this.offsetVisualizer.setParams({ 
          month: currentDate.getMonth() + 1,
          day: currentDate.getDate()
        });
        
        // Update other sliders
        monthSlider.value = (currentDate.getMonth() + 1).toString();
        daySlider.value = currentDate.getDate().toString();
        document.getElementById('offset-month-value')!.textContent = (currentDate.getMonth() + 1).toString();
        document.getElementById('offset-day-value')!.textContent = currentDate.getDate().toString();
        
        updateOffsetDisplay();
      }
    });
    
    weekSlider.addEventListener('input', () => {
      const week = parseInt(weekSlider.value);
      document.getElementById('offset-week-value')!.textContent = week.toString();
      
      if (this.offsetVisualizer) {
        const params = this.offsetVisualizer.getParams();
        // Calculate date for this week (week 1 = first week of January)
        const jan1 = new Date(params.year, 0, 1);
        const daysToAdd = (week - 1) * 7;
        const newDate = new Date(jan1.getTime() + daysToAdd * 24 * 60 * 60 * 1000);
        
        this.offsetVisualizer.setParams({
          month: newDate.getMonth() + 1,
          day: newDate.getDate()
        });
        
        // Update other sliders
        monthSlider.value = (newDate.getMonth() + 1).toString();
        daySlider.value = newDate.getDate().toString();
        document.getElementById('offset-month-value')!.textContent = (newDate.getMonth() + 1).toString();
        document.getElementById('offset-day-value')!.textContent = newDate.getDate().toString();
        
        updateOffsetDisplay();
      }
    });
    
    monthSlider.addEventListener('input', () => {
      const month = parseInt(monthSlider.value);
      document.getElementById('offset-month-value')!.textContent = month.toString();
      
      if (this.offsetVisualizer) {
        const params = this.offsetVisualizer.getParams();
        // Clamp day to valid range for new month
        const daysInMonth = new Date(params.year, month, 0).getDate();
        const clampedDay = Math.min(params.day, daysInMonth);
        
        this.offsetVisualizer.setParams({ month, day: clampedDay });
        
        // Update day slider if clamped
        if (clampedDay !== params.day) {
          daySlider.value = clampedDay.toString();
          document.getElementById('offset-day-value')!.textContent = clampedDay.toString();
        }
        
        // Update week slider
        const date = new Date(params.year, month - 1, clampedDay);
        const jan1 = new Date(params.year, 0, 1);
        const weekNum = Math.ceil(((date.getTime() - jan1.getTime()) / (24 * 60 * 60 * 1000) + jan1.getDay() + 1) / 7);
        weekSlider.value = weekNum.toString();
        document.getElementById('offset-week-value')!.textContent = weekNum.toString();
        
        updateOffsetDisplay();
      }
    });
    
    daySlider.addEventListener('input', () => {
      const day = parseInt(daySlider.value);
      document.getElementById('offset-day-value')!.textContent = day.toString();
      
      if (this.offsetVisualizer) {
        const params = this.offsetVisualizer.getParams();
        const daysInMonth = new Date(params.year, params.month, 0).getDate();
        const clampedDay = Math.min(day, daysInMonth);
        
        this.offsetVisualizer.setParams({ day: clampedDay });
        
        // Update week and dow sliders
        const date = new Date(params.year, params.month - 1, clampedDay);
        const dow = date.getDay() || 7;
        dowSlider.value = dow.toString();
        document.getElementById('offset-dow-value')!.textContent = dow.toString();
        
        const jan1 = new Date(params.year, 0, 1);
        const weekNum = Math.ceil(((date.getTime() - jan1.getTime()) / (24 * 60 * 60 * 1000) + jan1.getDay() + 1) / 7);
        weekSlider.value = weekNum.toString();
        document.getElementById('offset-week-value')!.textContent = weekNum.toString();
        
        updateOffsetDisplay();
      }
    });
    
    // Model parameter sliders
    const modelParams: { id: string; key: keyof TemperatureEstimatorParams }[] = [
      { id: 'offset-tEquator', key: 'T_EQUATOR' },
      { id: 'offset-aLat', key: 'A_LAT' },
      { id: 'offset-aYear', key: 'A_YEAR' },
      { id: 'offset-aDay', key: 'A_DAY' },
      { id: 'offset-h0', key: 'H0' },
      { id: 'offset-alpha', key: 'ALPHA' },
      { id: 'offset-beta', key: 'BETA' },
      { id: 'offset-lapseRate', key: 'LAPSE_RATE' }
    ];
    
    modelParams.forEach(({ id, key }) => {
      const slider = document.getElementById(id) as HTMLInputElement;
      const valueSpan = document.getElementById(`${id}-value`)!;
      
      slider.addEventListener('input', () => {
        const value = parseFloat(slider.value);
        valueSpan.textContent = value.toFixed(1);
        
        if (this.offsetVisualizer) {
          const params = this.offsetVisualizer.getParams();
          params.estimatorParams[key] = value;
          this.offsetVisualizer.setParams({ estimatorParams: params.estimatorParams });
          updateOffsetDisplay();
        }
      });
    });
    
    // Use GPS data button
    const useGpsBtn = document.getElementById('offset-use-gps-btn') as HTMLButtonElement;
    useGpsBtn.addEventListener('click', () => {
      if (this.currentGpsInfo && this.offsetVisualizer) {
        const { latitude, longitude, altitude, millis } = this.currentGpsInfo;
        const date = new Date(millis);
        
        this.offsetVisualizer.setParams({
          latitude,
          longitude,
          altitude,
          year: date.getUTCFullYear(),
          month: date.getUTCMonth() + 1,
          day: date.getUTCDate()
        });
        
        // Update all sliders
        latSlider.value = latitude.toString();
        lonSlider.value = longitude.toString();
        altSlider.value = altitude.toString();
        monthSlider.value = (date.getUTCMonth() + 1).toString();
        daySlider.value = date.getUTCDate().toString();
        
        document.getElementById('offset-lat-value')!.textContent = latitude.toFixed(2);
        document.getElementById('offset-lon-value')!.textContent = longitude.toFixed(2);
        document.getElementById('offset-alt-value')!.textContent = altitude.toFixed(0);
        document.getElementById('offset-month-value')!.textContent = (date.getUTCMonth() + 1).toString();
        document.getElementById('offset-day-value')!.textContent = date.getUTCDate().toString();
        
        // Update week and dow
        const dow = date.getDay() || 7;
        dowSlider.value = dow.toString();
        document.getElementById('offset-dow-value')!.textContent = dow.toString();
        
        const jan1 = new Date(date.getUTCFullYear(), 0, 1);
        const weekNum = Math.ceil(((date.getTime() - jan1.getTime()) / (24 * 60 * 60 * 1000) + jan1.getDay() + 1) / 7);
        weekSlider.value = weekNum.toString();
        document.getElementById('offset-week-value')!.textContent = weekNum.toString();
        
        updateOffsetDisplay();
      }
    });
    
    // Reset button
    const resetBtn = document.getElementById('offset-reset-btn') as HTMLButtonElement;
    resetBtn.addEventListener('click', () => {
      if (this.offsetVisualizer) {
        const currentYear = new Date().getFullYear();
        const defaults = {
          latitude: 41.22,
          longitude: -111.97,
          altitude: 1350,
          year: currentYear,
          month: 9,
          day: 16,
          estimatorParams: {
            T_EQUATOR: 32.0,
            A_LAT: 23.0,
            A_YEAR: 16.0,
            A_DAY: 10.0,
            H0: 3.0,
            ALPHA: 1.2,
            BETA: 1.0,
            LAPSE_RATE: 6.5,
            SUMMER_PEAK_DAY: null
          }
        };
        
        this.offsetVisualizer.setParams(defaults);
        
        // Update all sliders
        latSlider.value = '41.22';
        lonSlider.value = '-111.97';
        altSlider.value = '1350';
        monthSlider.value = '9';
        daySlider.value = '16';
        dowSlider.value = '1';
        weekSlider.value = '38';
        
        document.getElementById('offset-lat-value')!.textContent = '41.22';
        document.getElementById('offset-lon-value')!.textContent = '-111.97';
        document.getElementById('offset-alt-value')!.textContent = '1350';
        document.getElementById('offset-month-value')!.textContent = '9';
        document.getElementById('offset-day-value')!.textContent = '16';
        document.getElementById('offset-dow-value')!.textContent = '1';
        document.getElementById('offset-week-value')!.textContent = '38';
        
        // Reset model params
        (document.getElementById('offset-tEquator') as HTMLInputElement).value = '32.0';
        (document.getElementById('offset-aLat') as HTMLInputElement).value = '23.0';
        (document.getElementById('offset-aYear') as HTMLInputElement).value = '16.0';
        (document.getElementById('offset-aDay') as HTMLInputElement).value = '10.0';
        (document.getElementById('offset-h0') as HTMLInputElement).value = '3.0';
        (document.getElementById('offset-alpha') as HTMLInputElement).value = '1.2';
        (document.getElementById('offset-beta') as HTMLInputElement).value = '1.0';
        (document.getElementById('offset-lapseRate') as HTMLInputElement).value = '6.5';
        
        document.getElementById('offset-tEquator-value')!.textContent = '32.0';
        document.getElementById('offset-aLat-value')!.textContent = '23.0';
        document.getElementById('offset-aYear-value')!.textContent = '16.0';
        document.getElementById('offset-aDay-value')!.textContent = '10.0';
        document.getElementById('offset-h0-value')!.textContent = '3.0';
        document.getElementById('offset-alpha-value')!.textContent = '1.2';
        document.getElementById('offset-beta-value')!.textContent = '1.0';
        document.getElementById('offset-lapseRate-value')!.textContent = '6.5';
        
        updateOffsetDisplay();
      }
    });
  }

  /**
   * Update the temperature estimate display based on current GPS info and estimator params
   */
  private updateTemperatureEstimate(): void {
    if (!this.currentGpsInfo) {
      return;
    }

    const { latitude, longitude, millis, altitude } = this.currentGpsInfo;
    const result = this.temperatureEstimator.estimateTemperatureOffset(latitude, longitude, millis, altitude);
    this.lastEstimateResult = result;

    // Update GPS info display
    document.getElementById('gps-location')!.textContent = 
      `Lat: ${latitude.toFixed(4)}° | Lon: ${longitude.toFixed(4)}° | Alt: ${altitude.toFixed(0)}m`;
    document.getElementById('gps-datetime-utc')!.textContent = `UTC: ${result.utcDateTime}`;
    document.getElementById('gps-datetime-local')!.textContent = `Local: ${result.localDateTime}`;
    
    // Update estimated offset display
    document.getElementById('estimated-offset')!.textContent = 
      `${result.offsetK.toFixed(1)} K (${result.offsetF.toFixed(1)} °F)`;
    document.getElementById('estimated-sealevel')!.textContent = 
      `Sea Level: ${result.seaLevelTempF.toFixed(1)}°F | At Alt: ${result.altitudeTempF.toFixed(1)}°F`;
    document.getElementById('solar-info')!.textContent = 
      `Solar Hour: ${result.solarHour.toFixed(2)} | Lag: ${result.lag.toFixed(1)} hrs | Day: ${result.dayOfYear}`;
  }

  /**
   * Extract GPS info from track data and update the temperature estimator
   */
  private updateGpsInfoFromTrack(trackData: TrackPoint[]): void {
    if (trackData.length > 0) {
      // Use the first GPS point for location/time info
      const firstPoint = trackData[0];
      this.currentGpsInfo = {
        latitude: firstPoint.latitude,
        longitude: firstPoint.longitude,
        millis: firstPoint.millis,
        altitude: firstPoint.altitude
      };
      
      this.updateTemperatureEstimate();
    }
  }

  private setupPhysicsFilterControls() {
    // Toggle controls visibility
    this.toggleControlsBtn.addEventListener('click', () => {
      const filterControls = document.getElementById('filter-controls')!;
      const physicsControls = document.getElementById('physics-filter-controls')!;
      const orientationControls = document.getElementById('orientation-filter-controls')!;
      const offsetControls = document.getElementById('offset-filter-controls')!;
      const viewMode = (document.getElementById('view-mode') as HTMLSelectElement).value;
      
      if (viewMode === 'temperature') {
        const isVisible = filterControls.style.display !== 'none';
        filterControls.style.display = isVisible ? 'none' : 'block';
      } else if (viewMode === 'tempphysics') {
        const isVisible = physicsControls.style.display !== 'none';
        physicsControls.style.display = isVisible ? 'none' : 'block';
      } else if (viewMode === 'orientation') {
        const isVisible = orientationControls.style.display !== 'none';
        orientationControls.style.display = isVisible ? 'none' : 'block';
      } else if (viewMode === 'offset') {
        const isVisible = offsetControls.style.display !== 'none';
        offsetControls.style.display = isVisible ? 'none' : 'block';
      }
    });

    // Setup sliders
    const params: (keyof typeof this.physicsFilterParams)[] = [
      'pressureVar', 'deviceTempVar', 'tempProcessVar', 'biasProcessVar',
      'coolingProcessVar', 'rateProcessVar', 'heatingRate'
    ];

    params.forEach(param => {
      const slider = document.getElementById(`physics-${param}`) as HTMLInputElement;
      const valueSpan = document.getElementById(`physics-${param}-value`)!;
      
      slider.addEventListener('input', () => {
        const value = parseFloat(slider.value);
        this.physicsFilterParams[param] = value;
        if (param === 'heatingRate') {
          valueSpan.textContent = `${value.toFixed(3)} K/s`;
        } else {
          valueSpan.textContent = value.toFixed(value < 0.001 ? 5 : value < 1 ? 4 : value < 10 ? 1 : 0);
        }
      });
    });

    // Apply parameters and reprocess
    const applyBtn = document.getElementById('apply-physics-params-btn') as HTMLButtonElement;
    applyBtn.addEventListener('click', () => {
      if (this.trackText && this.sensorText) {
        this.processData(this.trackText, this.sensorText);
      }
    });

    // Reset to defaults
    const resetBtn = document.getElementById('reset-physics-params-btn') as HTMLButtonElement;
    resetBtn.addEventListener('click', () => {
      this.physicsFilterParams = {
        pressureVar: 100.0,
        deviceTempVar: 4.0,
        tempProcessVar: 0.01,
        biasProcessVar: 0.001,
        coolingProcessVar: 0.01,
        rateProcessVar: 0.1,
        heatingRate: 0.5
      };
      
      (document.getElementById('physics-pressureVar') as HTMLInputElement).value = '100';
      (document.getElementById('physics-deviceTempVar') as HTMLInputElement).value = '4.0';
      (document.getElementById('physics-tempProcessVar') as HTMLInputElement).value = '0.01';
      (document.getElementById('physics-biasProcessVar') as HTMLInputElement).value = '0.001';
      (document.getElementById('physics-coolingProcessVar') as HTMLInputElement).value = '0.01';
      (document.getElementById('physics-rateProcessVar') as HTMLInputElement).value = '0.1';
      (document.getElementById('physics-heatingRate') as HTMLInputElement).value = '0.5';
      
      document.getElementById('physics-pressureVar-value')!.textContent = '100';
      document.getElementById('physics-deviceTempVar-value')!.textContent = '4.0';
      document.getElementById('physics-tempProcessVar-value')!.textContent = '0.0100';
      document.getElementById('physics-biasProcessVar-value')!.textContent = '0.0010';
      document.getElementById('physics-coolingProcessVar-value')!.textContent = '0.0100';
      document.getElementById('physics-rateProcessVar-value')!.textContent = '0.100';
      document.getElementById('physics-heatingRate-value')!.textContent = '0.500 K/s';
      
      if (this.trackText && this.sensorText) {
        this.processData(this.trackText, this.sensorText);
      }
    });
  }

  private setupOrientationFilterControls() {
    // Setup sliders with logarithmic scale (slider range -3 to 1 maps to 0.001 to 10)
    const params: (keyof typeof this.orientationFilterParams)[] = [
      'gyroNoiseVar', 'gyroBiasNoiseVar', 'accelNoiseVar', 'magNoiseVar'
    ];

    params.forEach(param => {
      const slider = document.getElementById(`orientation-${param}`) as HTMLInputElement;
      const valueSpan = document.getElementById(`orientation-${param}-value`)!;
      
      slider.addEventListener('input', () => {
        // Convert log scale slider value to actual value: value = 10^(slider)
        const logValue = parseFloat(slider.value);
        const value = Math.pow(10, logValue);
        this.orientationFilterParams[param] = value;
        
        // Format based on magnitude
        if (value < 0.001) {
          valueSpan.textContent = value.toExponential(2);
        } else if (value < 0.01) {
          valueSpan.textContent = value.toFixed(5);
        } else if (value < 1) {
          valueSpan.textContent = value.toFixed(4);
        } else {
          valueSpan.textContent = value.toFixed(2);
        }
      });
    });

    // Apply parameters and reprocess
    const applyBtn = document.getElementById('apply-orientation-params-btn') as HTMLButtonElement;
    applyBtn.addEventListener('click', () => {
      if (this.trackText && this.sensorText) {
        this.processData(this.trackText, this.sensorText);
      }
    });

    // Reset to defaults
    const resetBtn = document.getElementById('reset-orientation-params-btn') as HTMLButtonElement;
    resetBtn.addEventListener('click', () => {
      this.orientationFilterParams = {
        gyroNoiseVar: 0.005,
        gyroBiasNoiseVar: 0.0001,
        accelNoiseVar: 1.0,
        magNoiseVar: 1.0
      };
      
      // Set slider values as log10 of the actual values
      (document.getElementById('orientation-gyroNoiseVar') as HTMLInputElement).value = String(Math.log10(0.005));
      (document.getElementById('orientation-gyroBiasNoiseVar') as HTMLInputElement).value = String(Math.log10(0.0001));
      (document.getElementById('orientation-accelNoiseVar') as HTMLInputElement).value = String(Math.log10(1.0));
      (document.getElementById('orientation-magNoiseVar') as HTMLInputElement).value = String(Math.log10(1.0));
      
      document.getElementById('orientation-gyroNoiseVar-value')!.textContent = '0.0050';
      document.getElementById('orientation-gyroBiasNoiseVar-value')!.textContent = '0.0001';
      document.getElementById('orientation-accelNoiseVar-value')!.textContent = '1.00';
      document.getElementById('orientation-magNoiseVar-value')!.textContent = '1.00';
      
      if (this.trackText && this.sensorText) {
        this.processData(this.trackText, this.sensorText);
      }
    });
  }

  private setupTempSourceSelector() {
    const tempSourceSelect = document.getElementById('temp-source') as HTMLSelectElement;
    
    tempSourceSelect.addEventListener('change', () => {
      this.tempSource = tempSourceSelect.value as 'BARO' | 'HUM' | 'MAG' | 'IMU';
      
      // Reprocess data with new temperature source
      if (this.trackText && this.sensorText) {
        this.processData(this.trackText, this.sensorText);
      }
    });
  }

  private setupDragDrop() {
    this.dropTarget.addEventListener('dragover', (e) => {
      e.preventDefault();
      this.dropZone.classList.add('active');
    });

    this.dropTarget.addEventListener('dragleave', (e) => {
      if (e.target === this.dropTarget) {
        this.dropZone.classList.remove('active');
      }
    });

    this.dropTarget.addEventListener('drop', async (e) => {
      e.preventDefault();
      this.dropZone.classList.remove('active');
      
      const items = Array.from(e.dataTransfer?.items || []);
      
      // Find folder with TRACK.CSV and SENSOR.CSV
      for (const item of items) {
        if (item.kind === 'file') {
          const entry = item.webkitGetAsEntry();
          if (entry && entry.isDirectory) {
            await this.loadFromFolder(entry as FileSystemDirectoryEntry);
            return;
          }
        }
      }
      
      this.status.textContent = 'Please drop a folder containing TRACK.CSV and SENSOR.CSV';
    });
  }

  private async loadFromFolder(folder: FileSystemDirectoryEntry) {
    this.status.textContent = 'Loading files...';
    this.dropZone.style.display = 'none';
    
    try {
      const trackFile = await this.findFile(folder, 'TRACK.CSV');
      const sensorFile = await this.findFile(folder, 'SENSOR.CSV');
      
      if (!trackFile || !sensorFile) {
        throw new Error('Could not find TRACK.CSV and SENSOR.CSV in folder');
      }
      
      const trackText = await this.readFile(trackFile);
      const sensorText = await this.readFile(sensorFile);
      
      await this.processData(trackText, sensorText);
      
    } catch (error) {
      this.status.textContent = `Error: ${(error as Error).message}`;
      console.error(error);
    }
  }

  private async findFile(folder: FileSystemDirectoryEntry, filename: string): Promise<File | null> {
    return new Promise((resolve) => {
      const reader = folder.createReader();
      reader.readEntries(async (entries) => {
        for (const entry of entries) {
          if (entry.isFile && entry.name.toUpperCase() === filename.toUpperCase()) {
            (entry as FileSystemFileEntry).file((file) => resolve(file));
            return;
          }
        }
        resolve(null);
      });
    });
  }

  private readFile(file: File): Promise<string> {
    return new Promise((resolve, reject) => {
      const reader = new FileReader();
      reader.onload = () => resolve(reader.result as string);
      reader.onerror = reject;
      reader.readAsText(file);
    });
  }

  private async loadDefaultData() {
    try {
      this.status.textContent = 'Loading default data...';
      
      const [trackResponse, sensorResponse] = await Promise.all([
        fetch('/data/TRACK.CSV'),
        fetch('/data/SENSOR.CSV')
      ]);
      
      this.trackText = await trackResponse.text();
      this.sensorText = await sensorResponse.text();
      
      await this.processData(this.trackText, this.sensorText);
      this.dropZone.style.display = 'none';
      
    } catch (error) {
      console.error('Could not load default data:', error);
      this.status.textContent = 'Drop a folder to begin';
    }
  }

  private async processData(trackText: string, sensorText: string) {
    this.status.textContent = 'Processing data...';
    
    // Store for reprocessing
    this.trackText = trackText;
    this.sensorText = sensorText;
    
    // Parse in worker to avoid blocking UI
    // For now, parse synchronously
    const trackParser = new TrackParser();
    const sensorParser = new SensorParser();
    
    trackParser.loadFromString(trackText);
    sensorParser.loadFromString(sensorText);
    
    const trackData = trackParser.getData();
    const sensorData = sensorParser.getData();
    
    // Update GPS info and temperature estimate from track data
    this.updateGpsInfoFromTrack(trackData);
    
    this.status.textContent = `Filtering ${sensorData.length} sensor readings...`;
    
    // Determine which filter to use based on view mode
    const viewMode = (document.getElementById('view-mode') as HTMLSelectElement).value as 'temperature' | 'tempphysics' | 'orientation';
    const usePhysicsFilter = viewMode === 'tempphysics';
    const useOrientationFilter = viewMode === 'orientation';
    
    console.log(`Processing data in ${viewMode} mode (useOrientationFilter: ${useOrientationFilter})`);
    
    // Run filter with current parameters
    let filter: FilterTemp | FilterTempPhysics | undefined;
    let orientationFilter: OrientationFilter | undefined;
    
    if (useOrientationFilter) {
      // Initialize orientation filter with user-configured noise parameters
      // IMU @ 13.33Hz (75ms), MAG @ 10Hz (100ms)
      // Balance between drift prevention and stability during dynamic motion
      orientationFilter = new OrientationFilter(
        this.orientationFilterParams.gyroNoiseVar,
        this.orientationFilterParams.gyroBiasNoiseVar,
        this.orientationFilterParams.accelNoiseVar,
        this.orientationFilterParams.magNoiseVar
      );
      console.log('Created OrientationFilter with parameters:', this.orientationFilterParams);
    } else if (usePhysicsFilter) {
      filter = new FilterTempPhysics(
        this.physicsFilterParams.pressureVar,
        this.physicsFilterParams.deviceTempVar,
        this.physicsFilterParams.tempProcessVar,
        this.physicsFilterParams.biasProcessVar,
        this.physicsFilterParams.coolingProcessVar,
        this.physicsFilterParams.rateProcessVar,
        this.physicsFilterParams.heatingRate
      );
    } else {
      filter = new FilterTemp(
        this.filterParams.pressureVar,
        this.filterParams.deviceTempVar,
        this.filterParams.tempProcessVar,
        this.filterParams.biasProcessVar,
        this.filterParams.rateProcessVar,
        this.filterParams.tempOffset
      );
    }
    
    const plotData: PlotPoint[] = [];
    
    let trackIndex = 0;
    let sensorIndex = 0;
    let lastTime = 0;
    let currentAltitude = 0;
    let currentPressure: number | undefined = undefined;  // Track most recent pressure
    let currentVelN: number = 0;  // Track GPS velocity North
    let currentVelE: number = 0;  // Track GPS velocity East
    let currentVelD: number = 0;  // Track GPS velocity Down
    let lastMag: { x: number; y: number; z: number } | undefined = undefined;  // Track last magnetometer reading
    
    while (trackIndex < trackData.length || sensorIndex < sensorData.length) {
      const nextTrack = trackIndex < trackData.length ? trackData[trackIndex] : null;
      const nextSensor = sensorIndex < sensorData.length ? sensorData[sensorIndex] : null;
      
      let currentTime: number;
      let dt: number;
      
      if (nextTrack && (!nextSensor || nextTrack.millis <= nextSensor.millis)) {
        currentTime = nextTrack.millis / 1000;
        dt = lastTime > 0 ? currentTime - lastTime : 0;
        currentAltitude = nextTrack.altitude;  // Update current altitude from GPS
        currentVelN = nextTrack.velN;  // Update GPS velocity
        currentVelE = nextTrack.velE;
        currentVelD = nextTrack.velD;
        trackIndex++;
      } else if (nextSensor) {
        currentTime = nextSensor.millis / 1000;
        dt = lastTime > 0 ? currentTime - lastTime : 0;
        
        // Update current pressure if this entry has it (BARO entries)
        if (nextSensor.pressure !== undefined) {
          currentPressure = nextSensor.pressure;
        }
        
        // Update magnetometer reading (MAG entries)
        if (nextSensor.mx !== undefined && nextSensor.my !== undefined && nextSensor.mz !== undefined) {
          const magMag = Math.sqrt(nextSensor.mx**2 + nextSensor.my**2 + nextSensor.mz**2);
          if (magMag > 0) {
            // Transform from FlySight body frame to NED frame, then normalize
            // FlySight: +X=West, +Y=Up, +Z=North
            // NED: +X=North, +Y=East, +Z=Down
            const mag_ned_x = nextSensor.mz;   // FlySight mz (North) → NED mx (North)
            const mag_ned_y = -nextSensor.mx;  // FlySight mx (West) → NED -my (negative East)
            const mag_ned_z = -nextSensor.my;  // FlySight my (Up) → NED -mz (negative Down)
            
            // Normalize to unit vector (standard practice for direction-only measurements)
            const magMag_ned = Math.sqrt(mag_ned_x**2 + mag_ned_y**2 + mag_ned_z**2);
            lastMag = { 
              x: mag_ned_x / magMag_ned, 
              y: mag_ned_y / magMag_ned, 
              z: mag_ned_z / magMag_ned 
            };
            
            // Debug: Log MAG entries with timing
            if (useOrientationFilter && plotData.length < 10) {
              console.log(`MAG entry at time ${currentTime.toFixed(3)}s:`, {
                raw_flysight: {x: nextSensor.mx, y: nextSensor.my, z: nextSensor.mz},
                ned: {x: mag_ned_x, y: mag_ned_y, z: mag_ned_z},
                normalized: lastMag,
                magnitude: magMag_ned.toFixed(4)
              });
            }
            
            // Apply magnetometer update immediately when new MAG data arrives
            if (useOrientationFilter && orientationFilter && orientationFilter.isInitialized()) {
              if (plotData.length < 10) {
                console.log(`  Applying MAG update at time ${currentTime.toFixed(3)}s`);
              }
              orientationFilter.updateMagnetometer(lastMag);
            }
          }
        }
        
        // Process orientation mode
        if (useOrientationFilter && orientationFilter) {
          // Initialize filter on first IMU reading if we have magnetometer data
          // Skip first 2 seconds to allow device to stabilize in hand or on desk
          if (!orientationFilter.isInitialized() &&
              currentTime >= 2.0 &&
              nextSensor.ax !== undefined && nextSensor.ay !== undefined && nextSensor.az !== undefined &&
              nextSensor.wx !== undefined && nextSensor.wy !== undefined && nextSensor.wz !== undefined &&
              lastMag !== undefined) {
            // Convert units: acceleration from g to m/s²
            // Transform from FlySight body frame to NED frame:
            // FlySight: +X=West, +Y=Up, +Z=North (when device faces north with top up)
            // NED: +X=North, +Y=East, +Z=Down
            // Transformation: [ax_ned, ay_ned, az_ned] = [az, -ax, -ay]
            const accel = { 
              x: nextSensor.az * 9.81,   // FlySight +Z (North) → NED +X (North)
              y: -nextSensor.ax * 9.81,  // FlySight +X (West) → NED -Y (negative East)
              z: -nextSensor.ay * 9.81   // FlySight +Y (Up) → NED -Z (negative Down)
            };
            // Gyro transformation (same as accel coordinate change)
            const gyro = {
              x: nextSensor.wz * Math.PI / 180,  // FlySight wz (about North) → NED wx
              y: -nextSensor.wx * Math.PI / 180, // FlySight wx (about West) → NED -wy  
              z: -nextSensor.wy * Math.PI / 180  // FlySight wy (about Up) → NED -wz
            };
            console.log(`Initializing filter at time ${currentTime.toFixed(3)}s`);
            console.log('  Raw accel FlySight frame (g):', {x: nextSensor.ax, y: nextSensor.ay, z: nextSensor.az});
            console.log('  Accel NED frame (m/s²):', accel);
            console.log('  Raw gyro (deg/s):', {x: nextSensor.wx, y: nextSensor.wy, z: nextSensor.wz});
            console.log('  Raw mag (gauss):', {x: lastMag.x, y: lastMag.y, z: lastMag.z});
            console.log('  Mag magnitude:', Math.sqrt(lastMag.x**2 + lastMag.y**2 + lastMag.z**2));
            orientationFilter.initialize(accel, lastMag, currentTime, gyro);
            const initEuler = orientationFilter.getEulerAngles();
            console.log('Filter initialized! Initial euler (rad):', initEuler, 'euler (deg):', {
              roll: initEuler.roll * 180 / Math.PI,
              pitch: initEuler.pitch * 180 / Math.PI,
              yaw: initEuler.yaw * 180 / Math.PI
            });
          }
          
          // Process IMU data (gyro + accel)
          if (orientationFilter.isInitialized() && dt > 0 &&
              nextSensor.wx !== undefined && nextSensor.wy !== undefined && nextSensor.wz !== undefined) {
            // NOTE: Raw sensor units from CSV are deg/s for gyro and g for accel
            // SensorParser does NOT convert them despite interface comments
            // Convert gyroscope from deg/s to rad/s AND transform to NED frame
            // FlySight: wx (about West), wy (about Up), wz (about North)
            // NED: wx (about North), wy (about East), wz (about Down)
            // Transformation: [wx_ned, wy_ned, wz_ned] = [wz, -wx, -wy]
            const gyro = { 
              x: nextSensor.wz * Math.PI / 180,   // FlySight wz (about North) → NED wx (about North)
              y: -nextSensor.wx * Math.PI / 180,  // FlySight wx (about West) → NED -wy (about negative East)
              z: -nextSensor.wy * Math.PI / 180   // FlySight wy (about Up) → NED -wz (about negative Down)
            };
            
            // Debug first few IMU readings with timing
            if (plotData.length < 5) {
              console.log(`IMU entry at time ${currentTime.toFixed(3)}s, dt=${dt.toFixed(4)}s`);
              console.log(`  Gyro raw (deg/s):`, {x: nextSensor.wx.toFixed(2), y: nextSensor.wy.toFixed(2), z: nextSensor.wz.toFixed(2)});
              console.log(`  Accel raw (g):`, {x: nextSensor.ax?.toFixed(3), y: nextSensor.ay?.toFixed(3), z: nextSensor.az?.toFixed(3)});
            }
            
            orientationFilter.predict(gyro, currentTime);
            
            // Update with accelerometer if available (convert g to m/s²)
            if (nextSensor.ax !== undefined && nextSensor.ay !== undefined && nextSensor.az !== undefined) {
              // Transform from FlySight body frame to NED frame
              const accel = { 
                x: nextSensor.az * 9.81,   // FlySight +Z (North) → NED +X (North)
                y: -nextSensor.ax * 9.81,  // FlySight +X (West) → NED -Y (negative East)
                z: -nextSensor.ay * 9.81   // FlySight +Y (Up) → NED -Z (negative Down)
              };
              orientationFilter.updateAccelerometer(accel);
            }
          }
          
         
          
          // Plot orientation data after processing each IMU entry (has gyro data)
          // Output every IMU sample (13.33Hz native sample rate)
          if (orientationFilter.isInitialized() && 
              nextSensor.wx !== undefined && nextSensor.wy !== undefined && nextSensor.wz !== undefined) {
            
            const euler = orientationFilter.getEulerAngles();
            const point = {
              time: currentTime,
              roll: euler.roll * 180 / Math.PI,   // Convert radians to degrees
              pitch: euler.pitch * 180 / Math.PI,
              yaw: euler.yaw * 180 / Math.PI,
              altitude: currentAltitude
            };
            
            // Debug: Log first few points and periodically
            if (plotData.length < 5 || plotData.length % 100 === 0) {
              console.log(`Plot point ${plotData.length}:`, point);
              const bias = orientationFilter.getGyroBias();
              const bx = (bias.x * 180 / Math.PI).toFixed(2);
              const by = (bias.y * 180 / Math.PI).toFixed(2);
              const bz = (bias.z * 180 / Math.PI).toFixed(2);
              console.log(`Bias(deg/s): x=${bx} y=${by} z=${bz}`);
              
              // Debug: Check if sensor values are stable
              if (nextSensor.ax && nextSensor.ay && nextSensor.az) {
                console.log(`Accel: ax=${nextSensor.ax.toFixed(3)} ay=${nextSensor.ay.toFixed(3)} az=${nextSensor.az.toFixed(3)}`);
              }
              if (nextSensor.wx !== undefined && nextSensor.wy !== undefined && nextSensor.wz !== undefined) {
                console.log(`Gyro: wx=${nextSensor.wx.toFixed(3)} wy=${nextSensor.wy.toFixed(3)} wz=${nextSensor.wz.toFixed(3)}`);
              }
            }
            
            plotData.push(point);
          }
        }
        
        // Get temperature from selected source
        let temperature: number | undefined;
        switch (this.tempSource) {
          case 'BARO':
            temperature = nextSensor.baroTemp;
            break;
          case 'HUM':
            temperature = nextSensor.humTemp;
            break;
          case 'MAG':
            temperature = nextSensor.magTemp;
            break;
          case 'IMU':
            temperature = nextSensor.imuTemp;
            break;
        }
        
        // Update temperature filter with pressure (use most recent pressure reading)
        if (!useOrientationFilter && filter && currentPressure !== undefined && currentAltitude !== 0 && dt > 0) {
          filter.updatePressure(currentPressure, currentAltitude, dt);
        }
        
        // Update temperature filter with device temperature from selected source
        if (!useOrientationFilter && filter && temperature !== undefined && dt > 0) {
          filter.updateDeviceTemperature(temperature + 273.15, dt);
        }
        
        // Update physics filter with airspeed if available
        if (usePhysicsFilter && filter && dt > 0) {
          (filter as FilterTempPhysics).updateAirspeed(currentVelN, currentVelE, currentVelD, dt);
          (filter as FilterTempPhysics).process(dt);
        }
        
        // Add temperature data to plot every 0.5 seconds
        const shouldPlot = Math.floor(currentTime * 2) % 1 === 0;
        const hasTemperatureData = !useOrientationFilter && filter && filter.isReady() && temperature !== undefined;
        
        if (shouldPlot && hasTemperatureData && filter) {
          // For FilterTempPhysics, tempOffset is now estimated in the state
          const usedTempOffset = usePhysicsFilter ? (filter as FilterTempPhysics).getTemperatureOffset() : this.filterParams.tempOffset;
          const coolingCoeff = usePhysicsFilter ? (filter as FilterTempPhysics).getCoolingCoefficient() : undefined;
          const heatingRate = usePhysicsFilter ? (filter as FilterTempPhysics).getHeatingRateParameter() : undefined;
          const bias = !usePhysicsFilter ? (filter as FilterTemp).getDeviceTemperatureBias() : 
                       (filter as FilterTempPhysics).getDeviceTemperatureBias();
          
          // Debug: Log first few values and periodically
          if (usePhysicsFilter && (plotData.length < 5 || plotData.length % 100 === 0)) {
            console.log(`Time: ${currentTime.toFixed(1)}s, α: ${coolingCoeff?.toFixed(6)}, β: ${heatingRate?.toFixed(3)}, offset: ${usedTempOffset.toFixed(2)}K, bias: ${bias.toFixed(2)}K, T_dev: ${temperature?.toFixed(1)}°C, T_out: ${filter.getOutsideTemperatureCelsius().toFixed(1)}°C`);
          }
          
          plotData.push({
            time: currentTime,
            deviceTemp: temperature !== undefined ? celsiusToFahrenheit(temperature) : undefined,
            filteredTemp: celsiusToFahrenheit(filter.getOutsideTemperatureCelsius()),
            standardTemp: currentAltitude > 0 ? getStandardTemperature(currentAltitude) : undefined,
            standardTempOffset: currentAltitude > 0 ? getStandardTemperatureWithOffset(currentAltitude, usedTempOffset) : undefined,
            bias: bias,
            altitude: currentAltitude,
            coolingCoeff: coolingCoeff,
            heatingRate: heatingRate
          });
        }
        
        sensorIndex++;
      } else {
        break;
      }
      
      lastTime = currentTime;
    }
    
    console.log(`Processing complete. Total plot points: ${plotData.length}`);
    if (useOrientationFilter) {
      console.log('Sample orientation points:', plotData.slice(0, 3));
    }
    
    this.plotter.setData(plotData);
    
    // Update orientation plotter if in orientation mode
    if (useOrientationFilter && this.orientationPlotter) {
      this.orientationPlotter.setData(plotData);
    }
    
    this.dataInfo.textContent = `${trackData.length} GPS points, ${sensorData.length} sensor readings`;
    this.resetBtn.disabled = false;
    
    // Update playback controller with orientation data
    if (useOrientationFilter && this.playbackController) {
      this.playbackController.setData(plotData);
    }
    
    if (useOrientationFilter && orientationFilter) {
      const initialized = orientationFilter.isInitialized();
      this.status.textContent = `Ready - ${plotData.length} points plotted (Orientation Filter: ${initialized ? 'Active' : 'Not Initialized'})`;
    } else {
      this.status.textContent = `Ready - ${plotData.length} points plotted`;
    }
  }
}

// Start app
new App();
