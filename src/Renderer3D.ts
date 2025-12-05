import * as THREE from 'three';
import { PlotPoint } from './main';

/**
 * 3D renderer for visualizing device orientation using Three.js
 * 
 * COORDINATE SYSTEM REFERENCE:
 * ============================
 * 
 * 1. FlySight Device Body Frame (per label on back of device):
 *    When device is held with FlySight text facing NORTH and top edge UP:
 *    +X = West (left side when viewing front, left when reading back label)
 *    +Y = Up (top edge)
 *    +Z = North (forward, toward FlySight text and green LED)
 *    Back label shows: Y↑, X←, Z⊗ (arrow into device, away from back label)
 * 
 *    MOUNTED CONFIGURATION (typical skydiving installation):
 *    When device is mounted on helmet/head and pilot faces NORTH:
 *    +X = East (device left side points to pilot's right)
 *    +Y = Up (device top edge points up)
 *    +Z = South (device front/text faces backward, away from pilot)
 *    (Device is rotated 180° around Y-axis relative to handheld orientation)
 * 
 * 2. NED Navigation Frame (used by EKF filter):
 *    +X = North
 *    +Y = East
 *    +Z = Down
 * 
 * 3. Three.js Visualization Frame:
 *    +X = Right (East)
 *    +Y = Up (opposite of Down)
 *    +Z = Toward viewer (opposite of North)
 * 
 * Coordinate Transformation (FlySight Body → NED):
 *    FlySight +X (West) → NED -Y (negative East)
 *    FlySight +Y (Up) → NED -Z (negative Down)
 *    FlySight +Z (North) → NED +X (North)
 * 
 * Euler Angles (from EKF, defined in NED):
 *    Roll  = Rotation about North axis (positive = right wing down)
 *    Pitch = Rotation about East axis (positive = nose down)
 *    Yaw   = Rotation about Down axis (positive = clockwise from above)
 * 
 * Device Face Labels (+X, +Y, +Z, etc.) show the FlySight body frame axes
 * World Labels (N, E, S, W, U, D) show the NED navigation frame
 */
export class Renderer3D {
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private renderer: THREE.WebGLRenderer;
  private deviceMesh: THREE.Group;
  private data: PlotPoint[] = [];
  
  constructor(private container: HTMLElement) {
    // Create scene
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x1e1e1e);
    
    // Create camera
    const aspect = container.clientWidth / container.clientHeight;
    this.camera = new THREE.PerspectiveCamera(50, aspect, 0.1, 1000);
    this.camera.position.set(3, 3, 5);
    this.camera.lookAt(0, 0, 0);
    
    // Create renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    this.renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(this.renderer.domElement);
    
    // Create device model
    this.deviceMesh = this.createDeviceModel();
    this.scene.add(this.deviceMesh);
    
    // Add lighting
    this.setupLighting();
    
    // Add axes helper
    const axesHelper = new THREE.AxesHelper(2);
    this.scene.add(axesHelper);
    
    // Add axis labels (N, E, S, W, U, D)
    this.addAxisLabels();
    
    // Add grid
    const gridHelper = new THREE.GridHelper(10, 10, 0x444444, 0x222222);
    this.scene.add(gridHelper);
    
    // Handle resize
    window.addEventListener('resize', () => this.handleResize());
    
    // Start render loop
    this.animate();
  }
  
  private createDeviceModel(): THREE.Group {
    const group = new THREE.Group();
    
    // Main body (box representing FlySight device)
    // Dimensions proportional to actual device: X=4, Y=4, Z=1 (thin rectangular device)
    // Scaled down for visualization: 0.8 × 0.8 × 0.2
    const bodyGeometry = new THREE.BoxGeometry(0.8, 0.8, 0.2);
    const bodyMaterial = new THREE.MeshPhongMaterial({ 
      color: 0x2563eb,
      shininess: 30
    });
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    group.add(body);
    
    // Add wireframe edges
    const edges = new THREE.EdgesGeometry(bodyGeometry);
    const line = new THREE.LineSegments(
      edges,
      new THREE.LineBasicMaterial({ color: 0xffffff, linewidth: 2 })
    );
    group.add(line);
    
    // Front face: FlySight text and green LED
    // Front is +Z face (toward the cone arrow)
    const frontTextCanvas = this.createFaceLabel('FLYSIGHT', '#ffffff', 24);
    const frontTexture = new THREE.CanvasTexture(frontTextCanvas);
    const frontSprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: frontTexture }));
    frontSprite.position.set(0, -0.15, 0.12);
    frontSprite.scale.set(0.6, 0.15, 1);
    group.add(frontSprite);
    
    // Green LED on front face (upper part)
    const ledGeometry = new THREE.CircleGeometry(0.03, 16);
    const ledMaterial = new THREE.MeshBasicMaterial({ color: 0x51cf66 });
    const led = new THREE.Mesh(ledGeometry, ledMaterial);
    led.position.set(0, 0.15, 0.11);
    group.add(led);
    
    // Add axis labels to each face
    // NOTE: FlySight body frame has +X pointing LEFT (when viewing front)
    // In Three.js, +X is to the right, so we need to swap the labels
    
    // +X face in Three.js (right side) = FlySight -X
    const minusXLabel = this.createAxisLabel('-X', '#51cf66');
    minusXLabel.position.set(0.45, 0, 0);
    minusXLabel.scale.set(1.0, 0.5, 1);
    group.add(minusXLabel);
    
    // -X face in Three.js (left side) = FlySight +X
    const plusXLabel = this.createAxisLabel('+X', '#51cf66');
    plusXLabel.position.set(-0.45, 0, 0);
    plusXLabel.scale.set(1.0, 0.5, 1);
    group.add(plusXLabel);
    
    // +Y face (top) - same in both frames
    const plusYLabel = this.createAxisLabel('+Y', '#4dabf7');
    plusYLabel.position.set(0, 0.45, 0);
    plusYLabel.scale.set(1.0, 0.5, 1);
    group.add(plusYLabel);
    
    // -Y face (bottom) - same in both frames
    const minusYLabel = this.createAxisLabel('-Y', '#4dabf7');
    minusYLabel.position.set(0, -0.45, 0);
    minusYLabel.scale.set(1.0, 0.5, 1);
    group.add(minusYLabel);
    
    // +Z face (front - where FlySight text is) - same in both frames
    const plusZLabel = this.createAxisLabel('+Z', '#ff6b6b');
    plusZLabel.position.set(0, 0.08, 0.12);
    plusZLabel.scale.set(0.6, 0.6, 1);
    group.add(plusZLabel);
    
    // -Z face (back with label diagram) - same in both frames
    const minusZLabel = this.createAxisLabel('-Z', '#ff6b6b');
    minusZLabel.position.set(0, 0, -0.12);
    minusZLabel.scale.set(1.0, 0.5, 1);
    group.add(minusZLabel);
    
    // Front indicator (arrow pointing forward)
    const arrowGeometry = new THREE.ConeGeometry(0.15, 0.4, 4);
    const arrowMaterial = new THREE.MeshPhongMaterial({ color: 0x51cf66 });
    const arrow = new THREE.Mesh(arrowGeometry, arrowMaterial);
    arrow.rotation.x = Math.PI / 2;
    arrow.position.z = 0.8;
    group.add(arrow);
    
    return group;
  }
  
  private createFaceLabel(text: string, color: string, fontSize: number): HTMLCanvasElement {
    const canvas = document.createElement('canvas');
    const context = canvas.getContext('2d')!;
    canvas.width = 256;
    canvas.height = 128;
    
    context.clearRect(0, 0, canvas.width, canvas.height);
    context.font = `Bold ${fontSize}px Arial`;
    context.fillStyle = color;
    context.textAlign = 'center';
    context.textBaseline = 'middle';
    context.fillText(text, 128, 64);
    
    return canvas;
  }
  
  private createAxisLabel(text: string, color: string): THREE.Sprite {
    const canvas = this.createFaceLabel(text, color, 48);
    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(0.25, 0.125, 1);
    return sprite;
  }
  
  private addAxisLabels(): void {
    // World frame labels: NED (North-East-Down) navigation frame
    // In Three.js Y-up visualization:
    //   -Z axis = North
    //   +X axis = East
    //   +Z axis = South
    //   -X axis = West
    //   +Y axis = Up (opposite of NED Down)
    //   -Y axis = Down
    
    const createTextSprite = (text: string, color: string): THREE.Sprite => {
      const canvas = document.createElement('canvas');
      const context = canvas.getContext('2d')!;
      canvas.width = 128;
      canvas.height = 128;
      
      context.clearRect(0, 0, canvas.width, canvas.height);
      context.font = 'Bold 64px Arial';
      context.fillStyle = color;
      context.textAlign = 'center';
      context.textBaseline = 'middle';
      context.fillText(text, 64, 64);
      
      const texture = new THREE.CanvasTexture(canvas);
      const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
      const sprite = new THREE.Sprite(spriteMaterial);
      sprite.scale.set(0.5, 0.5, 1);
      return sprite;
    };
    
    // North (NED +X, Three.js -Z)
    const northLabel = createTextSprite('N', '#ff6b6b');
    northLabel.position.set(0, 0, -3);
    this.scene.add(northLabel);
    
    // East (NED +Y, Three.js +X)
    const eastLabel = createTextSprite('E', '#51cf66');
    eastLabel.position.set(3, 0, 0);
    this.scene.add(eastLabel);
    
    // South (NED -X, Three.js +Z)
    const southLabel = createTextSprite('S', '#ff6b6b');
    southLabel.position.set(0, 0, 3);
    this.scene.add(southLabel);
    
    // West (NED -Y, Three.js -X)
    const westLabel = createTextSprite('W', '#51cf66');
    westLabel.position.set(-3, 0, 0);
    this.scene.add(westLabel);
    
    // Up (opposite of NED +Z, Three.js +Y)
    const upLabel = createTextSprite('U', '#4dabf7');
    upLabel.position.set(0, 3, 0);
    this.scene.add(upLabel);
    
    // Down (NED +Z, Three.js -Y)
    const downLabel = createTextSprite('D', '#4dabf7');
    downLabel.position.set(0, -3, 0);
    this.scene.add(downLabel);
  }
  
  private setupLighting(): void {
    // Ambient light
    const ambient = new THREE.AmbientLight(0xffffff, 0.6);
    this.scene.add(ambient);
    
    // Directional light
    const directional = new THREE.DirectionalLight(0xffffff, 0.8);
    directional.position.set(5, 10, 5);
    this.scene.add(directional);
    
    // Fill light
    const fill = new THREE.DirectionalLight(0x4dabf7, 0.3);
    fill.position.set(-5, 5, -5);
    this.scene.add(fill);
  }
  
  private handleResize(): void {
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;
    
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }
  
  private animate = (): void => {
    requestAnimationFrame(this.animate);
    this.renderer.render(this.scene, this.camera);
  };
  
  setData(data: PlotPoint[]): void {
    this.data = data;
  }
  
  setOrientation(roll: number, pitch: number, yaw: number): void {
    // Coordinate System Documentation:
    // ================================
    // FlySight Device Frame (body frame):
    //   +X: Right side of device (when looking at front/FlySight text)
    //   +Y: Up (perpendicular to front face, toward top edge)
    //   +Z: Forward (toward FlySight text and green LED)
    //
    // NED Frame (navigation frame used by EKF):
    //   +X: North
    //   +Y: East  
    //   +Z: Down
    //
    // Euler Angles from EKF (in NED frame):
    //   Roll:  Rotation about North axis (positive = right wing down)
    //   Pitch: Rotation about East axis (positive = nose down)
    //   Yaw:   Rotation about Down axis (positive = clockwise from above)
    //
    // Three.js Frame (visualization):
    //   +X: Right
    //   +Y: Up
    //   +Z: Out of screen (toward viewer)
    //
    // Mapping NED to Three.js visualization:
    //   North (NED +X) → -Z (Three.js)
    //   East  (NED +Y) → +X (Three.js)
    //   Down  (NED +Z) → -Y (Three.js)
    //
    // Convert degrees to radians and apply to device mesh
    // Three.js uses Y-up, right-handed coordinate system
    // Apply rotations in ZYX order (yaw, pitch, roll)
    this.deviceMesh.rotation.set(
      pitch * Math.PI / 180,  // X-axis rotation (pitch around East)
      yaw * Math.PI / 180,    // Y-axis rotation (yaw around Down->Up)
      roll * Math.PI / 180,   // Z-axis rotation (roll around North)
      'YXZ'                   // Yaw-Pitch-Roll order
    );
  }
  
  dispose(): void {
    this.renderer.dispose();
    window.removeEventListener('resize', () => this.handleResize());
  }
}
