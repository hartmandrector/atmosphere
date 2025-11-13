import { PlotPoint } from './main';
import { Renderer3D } from './Renderer3D';

/**
 * Controls playback of orientation data with the 3D renderer
 */
export class PlaybackController {
  private data: PlotPoint[] = [];
  private currentIndex: number = 0;
  private isPlaying: boolean = false;
  private animationFrameId: number | null = null;
  private lastTimestamp: number = 0;
  private playbackSpeed: number = 1.0;
  private accumulatedTime: number = 0;
  
  constructor(
    private renderer3D: Renderer3D,
    private playBtn: HTMLButtonElement,
    private pauseBtn: HTMLButtonElement,
    private stopBtn: HTMLButtonElement,
    private speedSlider: HTMLInputElement,
    private speedValue: HTMLSpanElement,
    private timeDisplay: HTMLSpanElement
  ) {
    this.setupControls();
  }
  
  private setupControls(): void {
    this.playBtn.addEventListener('click', () => this.play());
    this.pauseBtn.addEventListener('click', () => this.pause());
    this.stopBtn.addEventListener('click', () => this.stop());
    
    this.speedSlider.addEventListener('input', () => {
      this.playbackSpeed = parseFloat(this.speedSlider.value);
      this.speedValue.textContent = `${this.playbackSpeed.toFixed(1)}x`;
    });
    
    this.updateUI();
  }
  
  setData(data: PlotPoint[]): void {
    this.data = data.filter(p => 
      p.roll !== undefined && p.pitch !== undefined && p.yaw !== undefined
    );
    this.stop();
    this.renderer3D.setData(this.data);
    
    // Set initial orientation
    if (this.data.length > 0) {
      const first = this.data[0];
      this.renderer3D.setOrientation(
        first.roll || 0,
        first.pitch || 0,
        first.yaw || 0
      );
      this.updateTimeDisplay();
    }
  }
  
  play(): void {
    if (this.data.length === 0) return;
    
    if (this.currentIndex >= this.data.length - 1) {
      this.currentIndex = 0;
    }
    
    this.isPlaying = true;
    this.lastTimestamp = performance.now();
    this.accumulatedTime = 0;
    this.animate();
    this.updateUI();
  }
  
  pause(): void {
    this.isPlaying = false;
    if (this.animationFrameId !== null) {
      cancelAnimationFrame(this.animationFrameId);
      this.animationFrameId = null;
    }
    this.updateUI();
  }
  
  stop(): void {
    this.pause();
    this.currentIndex = 0;
    this.accumulatedTime = 0;
    if (this.data.length > 0) {
      const first = this.data[0];
      this.renderer3D.setOrientation(
        first.roll || 0,
        first.pitch || 0,
        first.yaw || 0
      );
    }
    this.updateTimeDisplay();
    this.updateUI();
  }
  
  private animate = (): void => {
    if (!this.isPlaying) return;
    
    const now = performance.now();
    const deltaMs = now - this.lastTimestamp;
    this.lastTimestamp = now;
    
    // Accumulate time with playback speed
    this.accumulatedTime += deltaMs * this.playbackSpeed;
    
    // Advance through frames as needed
    while (this.currentIndex < this.data.length - 1 && this.accumulatedTime > 0) {
      // Get time difference between samples in data (relative to first sample)
      const currentTime = this.data[this.currentIndex].time - this.data[0].time;
      const nextTime = this.data[this.currentIndex + 1].time - this.data[0].time;
      const dataTimeDelta = (nextTime - currentTime) * 1000; // Convert to ms
      
      if (this.accumulatedTime >= dataTimeDelta) {
        this.accumulatedTime -= dataTimeDelta;
        this.currentIndex++;
        
        const point = this.data[this.currentIndex];
        this.renderer3D.setOrientation(
          point.roll || 0,
          point.pitch || 0,
          point.yaw || 0
        );
        
        this.updateTimeDisplay();
      } else {
        break;
      }
    }
    
    if (this.currentIndex >= this.data.length - 1) {
      // Reached end
      this.pause();
    } else {
      this.animationFrameId = requestAnimationFrame(this.animate);
    }
  };
  
  private updateTimeDisplay(): void {
    if (this.data.length === 0) {
      this.timeDisplay.textContent = '0.0s / 0.0s';
      return;
    }
    
    const current = this.data[this.currentIndex];
    const last = this.data[this.data.length - 1];
    const currentTime = current.time - this.data[0].time;
    const totalTime = last.time - this.data[0].time;
    
    this.timeDisplay.textContent = `${currentTime.toFixed(1)}s / ${totalTime.toFixed(1)}s`;
  }
  
  private updateUI(): void {
    this.playBtn.disabled = this.isPlaying;
    this.pauseBtn.disabled = !this.isPlaying;
  }
}
