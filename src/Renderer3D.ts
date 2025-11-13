import * as THREE from 'three';
import { PlotPoint } from './main';

/**
 * 3D renderer for visualizing device orientation using Three.js
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
    const bodyGeometry = new THREE.BoxGeometry(0.6, 0.3, 1.2);
    const bodyMaterial = new THREE.MeshPhongMaterial({ 
      color: 0x2563eb,
      shininess: 30
    });
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    group.add(body);
    
    // Top indicator (to show orientation)
    const topGeometry = new THREE.BoxGeometry(0.5, 0.05, 0.5);
    const topMaterial = new THREE.MeshPhongMaterial({ color: 0xff6b6b });
    const top = new THREE.Mesh(topGeometry, topMaterial);
    top.position.y = 0.175;
    top.position.z = -0.3;
    group.add(top);
    
    // Front indicator (arrow)
    const arrowGeometry = new THREE.ConeGeometry(0.15, 0.4, 4);
    const arrowMaterial = new THREE.MeshPhongMaterial({ color: 0x51cf66 });
    const arrow = new THREE.Mesh(arrowGeometry, arrowMaterial);
    arrow.rotation.x = Math.PI / 2;
    arrow.position.z = 0.8;
    group.add(arrow);
    
    // Add wireframe edges
    const edges = new THREE.EdgesGeometry(bodyGeometry);
    const line = new THREE.LineSegments(
      edges,
      new THREE.LineBasicMaterial({ color: 0xffffff, linewidth: 2 })
    );
    group.add(line);
    
    return group;
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
    // Convert degrees to radians and apply to device mesh
    // Three.js uses Y-up, right-handed coordinate system
    // Apply rotations in ZYX order (yaw, pitch, roll)
    this.deviceMesh.rotation.set(
      pitch * Math.PI / 180,  // X-axis (pitch)
      yaw * Math.PI / 180,    // Y-axis (yaw)
      roll * Math.PI / 180,   // Z-axis (roll)
      'YXZ'                   // Yaw-Pitch-Roll order
    );
  }
  
  dispose(): void {
    this.renderer.dispose();
    window.removeEventListener('resize', () => this.handleResize());
  }
}
