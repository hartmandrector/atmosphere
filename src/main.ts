import { FilterTemp } from './filters/FilterTemp';
import { TrackParser, TrackPoint } from './parsers/TrackParser';
import { SensorParser, SensorData } from './parsers/SensorParser';

interface PlotPoint {
  time: number;
  deviceTemp?: number;  // in °F
  filteredTemp?: number;  // in °F
  standardTemp?: number;  // in °F - ISA standard atmosphere temperature
  standardTempOffset?: number;  // in °F - ISA standard + offset
  bias?: number;  // in K
  altitude?: number;  // in meters
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
  private minTime = 0;
  private maxTime = 0;
  private minTemp = 0;
  private maxTemp = 50;
  private minBias = 0;
  private maxBias = 1;
  private minAltitude = 0;
  private maxAltitude = 1000;
  private offsetX = 0;
  private offsetY = 0;
  private scale = 1;
  private isDragging = false;
  private lastX = 0;
  private lastY = 0;

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
      }
      this.minTemp = Math.floor(minT - 2);
      this.maxTemp = Math.ceil(maxT + 2);
      this.minBias = Math.floor(minB * 100) / 100 - 0.05;
      this.maxBias = Math.ceil(maxB * 100) / 100 + 0.05;
      this.minAltitude = Math.floor(minA / 100) * 100;
      this.maxAltitude = Math.ceil(maxA / 100) * 100;
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
    const marginRight = 120;
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
    this.drawGrid(ctx, marginLeft, marginTop, plotWidth, plotHeight, width - marginRight);
    
    // Plot data (temperature lines)
    this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'deviceTemp', '#ff6b6b', 2 / this.scale, 'temp');
    this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'filteredTemp', '#4dabf7', 2 / this.scale, 'temp');
    this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'standardTemp', '#51cf66', 2 / this.scale, 'temp');
    this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'standardTempOffset', '#22c55e', 2 / this.scale, 'temp');
    
    // Plot bias line (using right axis)
    this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'bias', '#ffd43b', 2 / this.scale, 'bias');
    
    // Plot altitude line (using far right axis)
    this.plotLine(ctx, marginLeft, marginTop, plotWidth, plotHeight, 'altitude', '#a78bfa', 1.5 / this.scale, 'altitude');
    
    ctx.restore();
  }

  private drawGrid(ctx: CanvasRenderingContext2D, x: number, y: number, 
                   width: number, height: number, rightAxisX: number) {
    const timeRange = this.maxTime - this.minTime;
    const tempRange = this.maxTemp - this.minTemp;
    const biasRange = this.maxBias - this.minBias;
    const altitudeRange = this.maxAltitude - this.minAltitude;
    
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
    
    // Horizontal grid (temperature - left axis)
    const tempStep = this.getStep(tempRange / 8);
    for (let temp = Math.ceil(this.minTemp / tempStep) * tempStep; temp <= this.maxTemp; temp += tempStep) {
      const py = y + height - ((temp - this.minTemp) / tempRange) * height;
      ctx.beginPath();
      ctx.moveTo(x, py);
      ctx.lineTo(x + width, py);
      ctx.stroke();
      
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
                   color: string, lineWidth: number, axis: 'temp' | 'bias' | 'altitude') {
    const timeRange = this.maxTime - this.minTime;
    let valueRange: number;
    let minValue: number;
    
    if (axis === 'bias') {
      valueRange = this.maxBias - this.minBias;
      minValue = this.minBias;
    } else if (axis === 'altitude') {
      valueRange = this.maxAltitude - this.minAltitude;
      minValue = this.minAltitude;
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
  
  // Temperature source selection
  private tempSource: 'BARO' | 'HUM' | 'MAG' | 'IMU' = 'BARO';
  
  // Store loaded data for reprocessing
  private trackText: string = '';
  private sensorText: string = '';

  constructor() {
    const canvas = document.getElementById('canvas') as HTMLCanvasElement;
    this.plotter = new Plotter(canvas);
    
    this.dropZone = document.getElementById('drop-zone')!;
    this.dropTarget = document.getElementById('drop-target')!;
    this.dataInfo = document.getElementById('data-info')!;
    this.status = document.getElementById('status')!;
    this.resetBtn = document.getElementById('reset-btn') as HTMLButtonElement;
    this.toggleControlsBtn = document.getElementById('toggle-controls-btn') as HTMLButtonElement;
    this.filterControls = document.getElementById('filter-controls')!;
    this.applyParamsBtn = document.getElementById('apply-params-btn') as HTMLButtonElement;
    this.resetParamsBtn = document.getElementById('reset-params-btn') as HTMLButtonElement;
    
    this.setupDragDrop();
    this.setupFilterControls();
    this.setupTempSourceSelector();
    this.resetBtn.addEventListener('click', () => this.plotter.resetView());
    
    // Load default data
    this.loadDefaultData();
  }

  private setupFilterControls() {
    // Toggle controls visibility
    this.toggleControlsBtn.addEventListener('click', () => {
      const isVisible = this.filterControls.style.display !== 'none';
      this.filterControls.style.display = isVisible ? 'none' : 'block';
    });

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
    
    this.status.textContent = `Filtering ${sensorData.length} sensor readings...`;
    
    // Run filter with current parameters
    const filter = new FilterTemp(
      this.filterParams.pressureVar,
      this.filterParams.deviceTempVar,
      this.filterParams.tempProcessVar,
      this.filterParams.biasProcessVar,
      this.filterParams.rateProcessVar,
      this.filterParams.tempOffset
    );
    const plotData: PlotPoint[] = [];
    
    let trackIndex = 0;
    let sensorIndex = 0;
    let lastTime = 0;
    let currentAltitude = 0;
    let currentPressure: number | undefined = undefined;  // Track most recent pressure
    
    while (trackIndex < trackData.length || sensorIndex < sensorData.length) {
      const nextTrack = trackIndex < trackData.length ? trackData[trackIndex] : null;
      const nextSensor = sensorIndex < sensorData.length ? sensorData[sensorIndex] : null;
      
      let currentTime: number;
      let dt: number;
      
      if (nextTrack && (!nextSensor || nextTrack.millis <= nextSensor.millis)) {
        currentTime = nextTrack.millis / 1000;
        dt = lastTime > 0 ? currentTime - lastTime : 0;
        currentAltitude = nextTrack.altitude;  // Update current altitude from GPS
        trackIndex++;
      } else if (nextSensor) {
        currentTime = nextSensor.millis / 1000;
        dt = lastTime > 0 ? currentTime - lastTime : 0;
        
        // Update current pressure if this entry has it (BARO entries)
        if (nextSensor.pressure !== undefined) {
          currentPressure = nextSensor.pressure;
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
        
        // Update filter with pressure (use most recent pressure reading)
        if (currentPressure !== undefined && currentAltitude !== 0 && dt > 0) {
          filter.updatePressure(currentPressure, currentAltitude, dt);
        }
        
        // Update filter with device temperature from selected source
        if (temperature !== undefined && dt > 0) {
          filter.updateDeviceTemperature(temperature + 273.15, dt);
        }
        
        // Add to plot data every 0.5 seconds
        if (filter.isReady() && temperature !== undefined && Math.floor(currentTime * 2) % 1 === 0) {
          plotData.push({
            time: currentTime,
            deviceTemp: celsiusToFahrenheit(temperature),
            filteredTemp: celsiusToFahrenheit(filter.getOutsideTemperatureCelsius()),
            standardTemp: currentAltitude > 0 ? getStandardTemperature(currentAltitude) : undefined,
            standardTempOffset: currentAltitude > 0 ? getStandardTemperatureWithOffset(currentAltitude, this.filterParams.tempOffset) : undefined,
            bias: filter.getDeviceTemperatureBias(),
            altitude: currentAltitude
          });
        }
        
        sensorIndex++;
      } else {
        break;
      }
      
      lastTime = currentTime;
    }
    
    this.plotter.setData(plotData);
    this.dataInfo.textContent = `${trackData.length} GPS points, ${sensorData.length} sensor readings`;
    this.resetBtn.disabled = false;
    this.status.textContent = `Ready - ${plotData.length} points plotted`;
  }
}

// Start app
new App();
