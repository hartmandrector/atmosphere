import { TemperatureEstimator, TemperatureEstimatorParams } from './utils/TemperatureEstimator';

export interface OffsetVisualizerOptions {
    yearCanvas: HTMLCanvasElement;
    monthCanvas: HTMLCanvasElement;
    dayCanvas: HTMLCanvasElement;
}

export interface OffsetVisualizerParams {
    latitude?: number;
    longitude?: number;
    altitude?: number;
    year?: number;
    month?: number;
    day?: number;
    estimatorParams?: Partial<TemperatureEstimatorParams>;
}

// Return type with all fields required
export interface OffsetVisualizerState {
    latitude: number;
    longitude: number;
    altitude: number;
    year: number;
    month: number;
    day: number;
    estimatorParams: TemperatureEstimatorParams;
}

/**
 * WeatherSpark-style temperature visualizer with three charts:
 * - Year chart (top): Full year view with daily high/low bands
 * - Month chart (bottom-left): Single month detail view
 * - Day chart (bottom-right): 24-hour temperature curve
 */
export class OffsetVisualizer {
    private yearCanvas: HTMLCanvasElement;
    private monthCanvas: HTMLCanvasElement;
    private dayCanvas: HTMLCanvasElement;
    
    private yearCtx: CanvasRenderingContext2D;
    private monthCtx: CanvasRenderingContext2D;
    private dayCtx: CanvasRenderingContext2D;
    
    private estimator: TemperatureEstimator;
    private estimatorParams: TemperatureEstimatorParams;
    
    // Current state
    private latitude: number = 40;
    private longitude: number = -105;
    private altitude: number = 0;
    private selectedMonth: number = 6; // 1-12
    private selectedDay: number = 15;
    private selectedYear: number = new Date().getFullYear();
    
    // Temperature range for scaling (in Fahrenheit)
    private minTemp: number = 14;
    private maxTemp: number = 104;
    
    // Colors matching WeatherSpark style
    private readonly HOT_COLOR = '#ff6b35';      // Hot band
    private readonly WARM_COLOR = '#ffb347';     // Warm band  
    private readonly COOL_COLOR = '#87ceeb';     // Cool band
    private readonly COLD_COLOR = '#4a90d9';     // Cold band
    private readonly LINE_COLOR = '#333';        // Temperature line
    private readonly GRID_COLOR = '#e0e0e0';     // Grid lines
    private readonly TEXT_COLOR = '#666';        // Axis labels
    
    // Support both constructor signatures for backwards compatibility
    constructor(yearCanvas: HTMLCanvasElement, monthCanvas: HTMLCanvasElement, dayCanvas: HTMLCanvasElement);
    constructor(options: OffsetVisualizerOptions);
    constructor(yearCanvasOrOptions: HTMLCanvasElement | OffsetVisualizerOptions, monthCanvas?: HTMLCanvasElement, dayCanvas?: HTMLCanvasElement) {
        if (yearCanvasOrOptions instanceof HTMLCanvasElement) {
            this.yearCanvas = yearCanvasOrOptions;
            this.monthCanvas = monthCanvas!;
            this.dayCanvas = dayCanvas!;
        } else {
            this.yearCanvas = yearCanvasOrOptions.yearCanvas;
            this.monthCanvas = yearCanvasOrOptions.monthCanvas;
            this.dayCanvas = yearCanvasOrOptions.dayCanvas;
        }
        
        this.yearCtx = this.yearCanvas.getContext('2d')!;
        this.monthCtx = this.monthCanvas.getContext('2d')!;
        this.dayCtx = this.dayCanvas.getContext('2d')!;
        
        this.estimatorParams = {
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
        this.estimator = new TemperatureEstimator(this.estimatorParams);
        
        this.setupCanvases();
    }
    
    private setupCanvases(): void {
        // Set canvas dimensions based on container
        this.resizeCanvases();
        window.addEventListener('resize', () => this.resizeCanvases());
    }
    
    private resizeCanvases(): void {
        // Year canvas - full width
        const yearParent = this.yearCanvas.parentElement;
        if (yearParent) {
            this.yearCanvas.width = yearParent.clientWidth || 800;
            this.yearCanvas.height = 200;
        }
        
        // Month and Day canvases - half width each
        const monthParent = this.monthCanvas.parentElement;
        if (monthParent) {
            this.monthCanvas.width = monthParent.clientWidth || 400;
            this.monthCanvas.height = 200;
        }
        
        const dayParent = this.dayCanvas.parentElement;
        if (dayParent) {
            this.dayCanvas.width = dayParent.clientWidth || 400;
            this.dayCanvas.height = 200;
        }
        
        this.render();
    }
    
    public setLocation(latitude: number, longitude: number): void {
        this.latitude = latitude;
        this.longitude = longitude;
        this.render();
    }
    
    public setDate(year: number, month: number, day: number): void {
        this.selectedYear = year;
        this.selectedMonth = month;
        this.selectedDay = day;
        this.render();
    }
    
    public getParams(): OffsetVisualizerState {
        return {
            latitude: this.latitude,
            longitude: this.longitude,
            altitude: this.altitude,
            year: this.selectedYear,
            month: this.selectedMonth,
            day: this.selectedDay,
            estimatorParams: { ...this.estimatorParams }
        };
    }
    
    public setParams(params: OffsetVisualizerParams): void {
        if (params.latitude !== undefined) this.latitude = params.latitude;
        if (params.longitude !== undefined) this.longitude = params.longitude;
        if (params.altitude !== undefined) this.altitude = params.altitude;
        if (params.year !== undefined) this.selectedYear = params.year;
        if (params.month !== undefined) this.selectedMonth = params.month;
        if (params.day !== undefined) this.selectedDay = params.day;
        if (params.estimatorParams) {
            this.estimatorParams = { ...this.estimatorParams, ...params.estimatorParams };
            this.estimator = new TemperatureEstimator(this.estimatorParams);
        }
        this.render();
    }
    
    public setEstimatorParams(params: Partial<TemperatureEstimatorParams>): void {
        this.estimatorParams = { ...this.estimatorParams, ...params };
        this.estimator = new TemperatureEstimator(this.estimatorParams);
        this.render();
    }
    
    public render(): void {
        this.calculateTemperatureRange();
        this.renderYearChart();
        this.renderMonthChart();
        this.renderDayChart();
    }
    
    private calculateTemperatureRange(): void {
        // Sample temperatures throughout the year to determine range
        let min = Infinity, max = -Infinity;
        
        for (let dayOfYear = 1; dayOfYear <= 365; dayOfYear++) {
            for (let hour = 0; hour < 24; hour += 6) {
                const date = this.dayOfYearToDate(dayOfYear);
                const timestamp = new Date(this.selectedYear, date.month - 1, date.day, hour).getTime();
                const result = this.estimator.estimateTemperatureOffset(this.latitude, this.longitude, timestamp, this.altitude);
                const temp = result.altitudeTempF;
                min = Math.min(min, temp);
                max = Math.max(max, temp);
            }
        }
        
        // Add padding
        this.minTemp = Math.floor(min / 5) * 5 - 5;
        this.maxTemp = Math.ceil(max / 5) * 5 + 5;
    }
    
    private dayOfYearToDate(dayOfYear: number): { month: number; day: number } {
        const date = new Date(this.selectedYear, 0, dayOfYear);
        return { month: date.getMonth() + 1, day: date.getDate() };
    }
    
    private dateToDayOfYear(month: number, day: number): number {
        const date = new Date(this.selectedYear, month - 1, day);
        const start = new Date(this.selectedYear, 0, 0);
        const diff = date.getTime() - start.getTime();
        return Math.floor(diff / (1000 * 60 * 60 * 24));
    }
    
    /**
     * Year Chart: Shows daily high/low temperature bands for the entire year
     * WeatherSpark style with color-coded temperature zones
     */
    private renderYearChart(): void {
        const ctx = this.yearCtx;
        const width = this.yearCanvas.width;
        const height = this.yearCanvas.height;
        
        // Clear canvas
        ctx.fillStyle = '#fff';
        ctx.fillRect(0, 0, width, height);
        
        const margin = { top: 30, right: 20, bottom: 40, left: 50 };
        const chartWidth = width - margin.left - margin.right;
        const chartHeight = height - margin.top - margin.bottom;
        
        // Draw title
        ctx.fillStyle = this.TEXT_COLOR;
        ctx.font = 'bold 14px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(`Temperature Throughout the Year at ${this.latitude.toFixed(1)}°N, ${Math.abs(this.longitude).toFixed(1)}°W`, width / 2, 18);
        
        // Draw temperature zones background
        this.drawTemperatureZones(ctx, margin.left, margin.top, chartWidth, chartHeight);
        
        // Collect daily high/low data
        const dailyData: { high: number; low: number }[] = [];
        
        for (let dayOfYear = 1; dayOfYear <= 365; dayOfYear++) {
            let dayHigh = -Infinity, dayLow = Infinity;
            
            for (let hour = 0; hour < 24; hour++) {
                const date = this.dayOfYearToDate(dayOfYear);
                const timestamp = new Date(this.selectedYear, date.month - 1, date.day, hour).getTime();
                const result = this.estimator.estimateTemperatureOffset(this.latitude, this.longitude, timestamp, this.altitude);
                const temp = result.altitudeTempF;
                dayHigh = Math.max(dayHigh, temp);
                dayLow = Math.min(dayLow, temp);
            }
            
            dailyData.push({ high: dayHigh, low: dayLow });
        }
        
        // Draw the band (area between high and low)
        const tempToY = (temp: number) => {
            return margin.top + chartHeight - (temp - this.minTemp) / (this.maxTemp - this.minTemp) * chartHeight;
        };
        
        const dayToX = (day: number) => {
            return margin.left + (day - 1) / 364 * chartWidth;
        };
        
        // Draw filled band
        ctx.beginPath();
        ctx.moveTo(dayToX(1), tempToY(dailyData[0].high));
        
        // Top edge (highs)
        for (let i = 0; i < dailyData.length; i++) {
            ctx.lineTo(dayToX(i + 1), tempToY(dailyData[i].high));
        }
        
        // Bottom edge (lows) - reverse direction
        for (let i = dailyData.length - 1; i >= 0; i--) {
            ctx.lineTo(dayToX(i + 1), tempToY(dailyData[i].low));
        }
        
        ctx.closePath();
        
        // Create gradient for the band
        const gradient = ctx.createLinearGradient(0, margin.top, 0, margin.top + chartHeight);
        gradient.addColorStop(0, 'rgba(255, 107, 53, 0.6)');   // Hot
        gradient.addColorStop(0.3, 'rgba(255, 179, 71, 0.6)'); // Warm
        gradient.addColorStop(0.6, 'rgba(135, 206, 235, 0.6)');// Cool
        gradient.addColorStop(1, 'rgba(74, 144, 217, 0.6)');   // Cold
        ctx.fillStyle = gradient;
        ctx.fill();
        
        // Draw high line
        ctx.beginPath();
        ctx.strokeStyle = this.HOT_COLOR;
        ctx.lineWidth = 1.5;
        for (let i = 0; i < dailyData.length; i++) {
            if (i === 0) ctx.moveTo(dayToX(i + 1), tempToY(dailyData[i].high));
            else ctx.lineTo(dayToX(i + 1), tempToY(dailyData[i].high));
        }
        ctx.stroke();
        
        // Draw low line
        ctx.beginPath();
        ctx.strokeStyle = this.COLD_COLOR;
        ctx.lineWidth = 1.5;
        for (let i = 0; i < dailyData.length; i++) {
            if (i === 0) ctx.moveTo(dayToX(i + 1), tempToY(dailyData[i].low));
            else ctx.lineTo(dayToX(i + 1), tempToY(dailyData[i].low));
        }
        ctx.stroke();
        
        // Draw selected day marker
        const selectedDayOfYear = this.dateToDayOfYear(this.selectedMonth, this.selectedDay);
        ctx.beginPath();
        ctx.strokeStyle = '#333';
        ctx.lineWidth = 2;
        ctx.setLineDash([4, 4]);
        ctx.moveTo(dayToX(selectedDayOfYear), margin.top);
        ctx.lineTo(dayToX(selectedDayOfYear), margin.top + chartHeight);
        ctx.stroke();
        ctx.setLineDash([]);
        
        // Draw axes
        this.drawYearAxes(ctx, margin, chartWidth, chartHeight);
    }
    
    private drawTemperatureZones(ctx: CanvasRenderingContext2D, x: number, y: number, width: number, height: number): void {
        const tempRange = this.maxTemp - this.minTemp;
        
        // Define temperature thresholds
        const zones = [
            { temp: 86, color: 'rgba(255, 107, 53, 0.15)' },   // Hot (>86°F / 30°C)
            { temp: 68, color: 'rgba(255, 179, 71, 0.15)' },   // Warm (68-86°F / 20-30°C)
            { temp: 50, color: 'rgba(135, 206, 235, 0.15)' },  // Cool (50-68°F / 10-20°C)
            { temp: 32, color: 'rgba(74, 144, 217, 0.15)' },   // Cold (32-50°F / 0-10°C)
            { temp: -Infinity, color: 'rgba(100, 149, 237, 0.15)' }  // Freezing (<32°F / 0°C)
        ];
        
        let prevY = y;
        for (let i = 0; i < zones.length - 1; i++) {
            const topTemp = i === 0 ? this.maxTemp : zones[i].temp;
            const bottomTemp = zones[i + 1].temp === -Infinity ? this.minTemp : zones[i + 1].temp;
            
            if (topTemp > this.minTemp && bottomTemp < this.maxTemp) {
                const clampedTop = Math.min(topTemp, this.maxTemp);
                const clampedBottom = Math.max(bottomTemp, this.minTemp);
                
                const zoneY = y + (this.maxTemp - clampedTop) / tempRange * height;
                const zoneHeight = (clampedTop - clampedBottom) / tempRange * height;
                
                ctx.fillStyle = zones[i].color;
                ctx.fillRect(x, zoneY, width, zoneHeight);
            }
        }
    }
    
    private drawYearAxes(ctx: CanvasRenderingContext2D, margin: { top: number; right: number; bottom: number; left: number }, chartWidth: number, chartHeight: number): void {
        const months = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec'];
        const monthStarts = [1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335];
        
        ctx.fillStyle = this.TEXT_COLOR;
        ctx.font = '11px sans-serif';
        ctx.textAlign = 'center';
        
        // Month labels
        for (let i = 0; i < 12; i++) {
            const x = margin.left + (monthStarts[i] - 1) / 364 * chartWidth;
            const nextStart = i < 11 ? monthStarts[i + 1] : 366;
            const midX = margin.left + ((monthStarts[i] + nextStart) / 2 - 1) / 364 * chartWidth;
            
            // Vertical grid line at month start
            ctx.beginPath();
            ctx.strokeStyle = this.GRID_COLOR;
            ctx.lineWidth = 1;
            ctx.moveTo(x, margin.top);
            ctx.lineTo(x, margin.top + chartHeight);
            ctx.stroke();
            
            // Month label
            ctx.fillText(months[i], midX, margin.top + chartHeight + 15);
        }
        
        // Y-axis (temperature)
        ctx.textAlign = 'right';
        const tempStep = 20;
        for (let temp = Math.ceil(this.minTemp / tempStep) * tempStep; temp <= this.maxTemp; temp += tempStep) {
            const y = margin.top + chartHeight - (temp - this.minTemp) / (this.maxTemp - this.minTemp) * chartHeight;
            
            // Horizontal grid line
            ctx.beginPath();
            ctx.strokeStyle = this.GRID_COLOR;
            ctx.moveTo(margin.left, y);
            ctx.lineTo(margin.left + chartWidth, y);
            ctx.stroke();
            
            // Temperature label
            ctx.fillText(`${temp}°F`, margin.left - 5, y + 4);
        }
    }
    
    /**
     * Month Chart: Shows daily temperature variation for the selected month
     */
    private renderMonthChart(): void {
        const ctx = this.monthCtx;
        const width = this.monthCanvas.width;
        const height = this.monthCanvas.height;
        
        // Clear canvas
        ctx.fillStyle = '#fff';
        ctx.fillRect(0, 0, width, height);
        
        const margin = { top: 30, right: 20, bottom: 40, left: 50 };
        const chartWidth = width - margin.left - margin.right;
        const chartHeight = height - margin.top - margin.bottom;
        
        const monthNames = ['January', 'February', 'March', 'April', 'May', 'June',
                           'July', 'August', 'September', 'October', 'November', 'December'];
        
        // Draw title
        ctx.fillStyle = this.TEXT_COLOR;
        ctx.font = 'bold 14px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(`${monthNames[this.selectedMonth - 1]} ${this.selectedYear}`, width / 2, 18);
        
        // Draw temperature zones
        this.drawTemperatureZones(ctx, margin.left, margin.top, chartWidth, chartHeight);
        
        // Get days in month
        const daysInMonth = new Date(this.selectedYear, this.selectedMonth, 0).getDate();
        
        const tempToY = (temp: number) => {
            return margin.top + chartHeight - (temp - this.minTemp) / (this.maxTemp - this.minTemp) * chartHeight;
        };
        
        // Collect hourly data for sawtooth pattern
        const hourlyData: { day: number; hour: number; temp: number }[] = [];
        
        for (let day = 1; day <= daysInMonth; day++) {
            for (let hour = 0; hour < 24; hour += 2) { // Sample every 2 hours for smoother line
                const timestamp = new Date(this.selectedYear, this.selectedMonth - 1, day, hour).getTime();
                const result = this.estimator.estimateTemperatureOffset(this.latitude, this.longitude, timestamp, this.altitude);
                const temp = result.altitudeTempF;
                hourlyData.push({ day, hour, temp });
            }
        }
        
        // Draw the sawtooth temperature line
        ctx.beginPath();
        ctx.strokeStyle = this.LINE_COLOR;
        ctx.lineWidth = 1.5;
        
        for (let i = 0; i < hourlyData.length; i++) {
            const data = hourlyData[i];
            const x = margin.left + ((data.day - 1) + data.hour / 24) / daysInMonth * chartWidth;
            const y = tempToY(data.temp);
            
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
        
        // Draw selected day marker
        ctx.beginPath();
        ctx.strokeStyle = '#333';
        ctx.lineWidth = 2;
        ctx.setLineDash([4, 4]);
        const selectedX = margin.left + (this.selectedDay - 0.5) / daysInMonth * chartWidth;
        ctx.moveTo(selectedX, margin.top);
        ctx.lineTo(selectedX, margin.top + chartHeight);
        ctx.stroke();
        ctx.setLineDash([]);
        
        // Draw axes
        ctx.fillStyle = this.TEXT_COLOR;
        ctx.font = '11px sans-serif';
        ctx.textAlign = 'center';
        
        // Day labels (every 5 days)
        for (let day = 1; day <= daysInMonth; day += 5) {
            const x = margin.left + (day - 0.5) / daysInMonth * chartWidth;
            ctx.fillText(`${day}`, x, margin.top + chartHeight + 15);
            
            // Vertical grid
            ctx.beginPath();
            ctx.strokeStyle = this.GRID_COLOR;
            ctx.moveTo(x, margin.top);
            ctx.lineTo(x, margin.top + chartHeight);
            ctx.stroke();
        }
        
        // Y-axis
        ctx.textAlign = 'right';
        const tempStep = 20;
        for (let temp = Math.ceil(this.minTemp / tempStep) * tempStep; temp <= this.maxTemp; temp += tempStep) {
            const y = tempToY(temp);
            
            ctx.beginPath();
            ctx.strokeStyle = this.GRID_COLOR;
            ctx.moveTo(margin.left, y);
            ctx.lineTo(margin.left + chartWidth, y);
            ctx.stroke();
            
            ctx.fillText(`${temp}°F`, margin.left - 5, y + 4);
        }
    }
    
    /**
     * Day Chart: Shows 24-hour temperature curve with confidence band
     */
    private renderDayChart(): void {
        const ctx = this.dayCtx;
        const width = this.dayCanvas.width;
        const height = this.dayCanvas.height;
        
        // Clear canvas
        ctx.fillStyle = '#fff';
        ctx.fillRect(0, 0, width, height);
        
        const margin = { top: 30, right: 20, bottom: 40, left: 50 };
        const chartWidth = width - margin.left - margin.right;
        const chartHeight = height - margin.top - margin.bottom;
        
        const monthNames = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun',
                           'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec'];
        
        // Draw title
        ctx.fillStyle = this.TEXT_COLOR;
        ctx.font = 'bold 14px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(`${monthNames[this.selectedMonth - 1]} ${this.selectedDay}, ${this.selectedYear}`, width / 2, 18);
        
        // Draw temperature zones
        this.drawTemperatureZones(ctx, margin.left, margin.top, chartWidth, chartHeight);
        
        const tempToY = (temp: number) => {
            return margin.top + chartHeight - (temp - this.minTemp) / (this.maxTemp - this.minTemp) * chartHeight;
        };
        
        const hourToX = (hour: number) => {
            return margin.left + hour / 24 * chartWidth;
        };
        
        // Collect hourly temperatures
        const hourlyTemps: number[] = [];
        for (let hour = 0; hour <= 24; hour++) {
            const timestamp = new Date(this.selectedYear, this.selectedMonth - 1, this.selectedDay, hour % 24).getTime();
            const result = this.estimator.estimateTemperatureOffset(this.latitude, this.longitude, timestamp, this.altitude);
            hourlyTemps.push(result.altitudeTempF);
        }
        
        // Draw confidence band (±4°F uncertainty visualization)
        const uncertainty = 4;
        ctx.beginPath();
        ctx.moveTo(hourToX(0), tempToY(hourlyTemps[0] + uncertainty));
        for (let hour = 0; hour <= 24; hour++) {
            ctx.lineTo(hourToX(hour), tempToY(hourlyTemps[hour] + uncertainty));
        }
        for (let hour = 24; hour >= 0; hour--) {
            ctx.lineTo(hourToX(hour), tempToY(hourlyTemps[hour] - uncertainty));
        }
        ctx.closePath();
        ctx.fillStyle = 'rgba(100, 149, 237, 0.2)';
        ctx.fill();
        
        // Draw main temperature curve
        ctx.beginPath();
        ctx.strokeStyle = this.LINE_COLOR;
        ctx.lineWidth = 2;
        for (let hour = 0; hour <= 24; hour++) {
            const x = hourToX(hour);
            const y = tempToY(hourlyTemps[hour]);
            if (hour === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
        
        // Draw current time marker (noon as reference)
        ctx.beginPath();
        ctx.strokeStyle = '#e74c3c';
        ctx.lineWidth = 2;
        ctx.setLineDash([4, 4]);
        ctx.moveTo(hourToX(12), margin.top);
        ctx.lineTo(hourToX(12), margin.top + chartHeight);
        ctx.stroke();
        ctx.setLineDash([]);
        
        // Draw temperature label at noon
        const noonTemp = hourlyTemps[12];
        ctx.fillStyle = '#e74c3c';
        ctx.font = 'bold 12px sans-serif';
        ctx.textAlign = 'left';
        ctx.fillText(`${noonTemp.toFixed(1)}°F`, hourToX(12) + 5, tempToY(noonTemp) - 5);
        
        // Draw axes
        ctx.fillStyle = this.TEXT_COLOR;
        ctx.font = '11px sans-serif';
        ctx.textAlign = 'center';
        
        // Hour labels
        for (let hour = 0; hour <= 24; hour += 6) {
            const x = hourToX(hour);
            const label = hour === 0 ? '12am' : hour === 6 ? '6am' : hour === 12 ? '12pm' : hour === 18 ? '6pm' : '12am';
            ctx.fillText(label, x, margin.top + chartHeight + 15);
            
            ctx.beginPath();
            ctx.strokeStyle = this.GRID_COLOR;
            ctx.moveTo(x, margin.top);
            ctx.lineTo(x, margin.top + chartHeight);
            ctx.stroke();
        }
        
        // Y-axis
        ctx.textAlign = 'right';
        const tempStep = 20;
        for (let temp = Math.ceil(this.minTemp / tempStep) * tempStep; temp <= this.maxTemp; temp += tempStep) {
            const y = tempToY(temp);
            
            ctx.beginPath();
            ctx.strokeStyle = this.GRID_COLOR;
            ctx.moveTo(margin.left, y);
            ctx.lineTo(margin.left + chartWidth, y);
            ctx.stroke();
            
            ctx.fillText(`${temp}°F`, margin.left - 5, y + 4);
        }
        
        // Draw min/max labels
        const minTemp = Math.min(...hourlyTemps);
        const maxTemp = Math.max(...hourlyTemps);
        const minHour = hourlyTemps.indexOf(minTemp);
        const maxHour = hourlyTemps.indexOf(maxTemp);
        
        ctx.fillStyle = this.COLD_COLOR;
        ctx.font = 'bold 11px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(`Low: ${minTemp.toFixed(1)}°F`, hourToX(minHour), tempToY(minTemp) + 15);
        
        ctx.fillStyle = this.HOT_COLOR;
        ctx.fillText(`High: ${maxTemp.toFixed(1)}°F`, hourToX(maxHour), tempToY(maxTemp) - 8);
    }
}
