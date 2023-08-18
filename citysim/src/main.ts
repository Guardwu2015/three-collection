import GUI from 'lil-gui'
import {
  AmbientLight,
  AnimationMixer,
  AxesHelper,
  BoxGeometry,
  Clock,
  GridHelper,
  LoadingManager,
  Mesh,
  MeshLambertMaterial,
  MeshStandardMaterial,
  PCFSoftShadowMap,
  PerspectiveCamera,
  PlaneGeometry,
  PointLight,
  PointLightHelper,
  Scene,
  Vector2,
  Vector3,
  WebGLRenderer,
} from 'three'
import TWEEN from "@tweenjs/tween.js";
import { DragControls } from 'three/examples/jsm/controls/DragControls'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls'
import Stats from 'three/examples/jsm/libs/stats.module'

import { EffectComposer } from "three/examples/jsm/postprocessing/EffectComposer.js"
import { RenderPass } from "three/examples/jsm/postprocessing/RenderPass.js"
import { UnrealBloomPass } from "three/examples/jsm/postprocessing/UnrealBloomPass.js"


import * as animations from './helpers/animations'
import { toggleFullScreen } from './helpers/fullscreen'
import { resizeRendererToDisplaySize } from './helpers/responsiveness'
import './style.css'

const CANVAS_ID = 'scene'

const animation = { enabled: false, play: true }

class Main {
  public canvas?: HTMLElement
  public renderer?: WebGLRenderer
  public scene?: Scene
  public loadingManager?: LoadingManager
  public ambientLight?: AmbientLight
  public pointLight?: PointLight
  public cube?: Mesh
  public camera?: PerspectiveCamera
  public cameraControls?: OrbitControls
  public dragControls?: DragControls
  public axesHelper?: AxesHelper
  public pointLightHelper?: PointLightHelper
  public clock?: Clock
  public stats?: Stats
  public gui?: GUI
  public bloomComposer?: EffectComposer
  public mixers: AnimationMixer[] = [] 

  constructor() {
    this.init()
  }
  init() {
    this.initScene()
    this.initRenderer()
    this.initCamera()
    this.initLights()
    this.initLoadingManager()

    this.addObjects()
    this.addControls()
    this.addHelpers()
    this.addGUI()
    this.addClock()
    this.addStats()
    this.addBloomComposer()
  
    // Full screen
    window.addEventListener('dblclick', (event) => {
      if (event.target === this.canvas) {
        toggleFullScreen(this.canvas)
      }
    })
  
    this.animate()
  }

  animate() {
    requestAnimationFrame(this.animate.bind(this))
  
    this.stats?.update()
  
    if (animation.enabled && animation.play) {
      if (this.cube && this.clock) {
        animations.rotate(this.cube, this.clock, Math.PI / 3)
        animations.bounce(this.cube, this.clock, 1, 0.5, 0.5)
      }
    }
  
    if (resizeRendererToDisplaySize(this.renderer!)) {
      const canvas = this.renderer!.domElement
      this.camera!.aspect = canvas.clientWidth / canvas.clientHeight
      this.camera!.updateProjectionMatrix()
    }
  
    this.cameraControls?.update()
  
    TWEEN.update();
  
    if (this.mixers.length && this.clock) {
      const delta = this.clock.getDelta();
      for (const mixer of this.mixers) {
        mixer.update(delta);
      }
    }
  
    this.renderer?.render(this.scene!, this.camera!)
    // bloomComposer.render()
  }
  
  initScene() {
    this.scene = new Scene()
  }
  
  initRenderer() {
    this.canvas = document.querySelector(`canvas#${CANVAS_ID}`)!
    this.renderer = new WebGLRenderer({ canvas: this.canvas, antialias: true, alpha: true })
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    this.renderer.shadowMap.enabled = true
    this.renderer.shadowMap.type = PCFSoftShadowMap
  }
  
  initCamera() {
    this.camera = new PerspectiveCamera(50, this.canvas!.clientWidth / this.canvas!.clientHeight, 0.1, 100)
    this.camera.position.set(2, 2, 5)
  
    this.cameraControls = new OrbitControls(this.camera, this.canvas)
    this.cameraControls.target = new Vector3(0, 0, 0)
    this.cameraControls.enableDamping = true
    this.cameraControls.autoRotate = false
    this.cameraControls.update()
  }

  initLights() {
    this.ambientLight = new AmbientLight('white', 0.4)
    this.pointLight = new PointLight('#ffdca8', 1.2, 100)
    this.pointLight.position.set(-2, 3, 3)
    this.pointLight.castShadow = true
    this.pointLight.shadow.radius = 4
    this.pointLight.shadow.camera.near = 0.5
    this.pointLight.shadow.camera.far = 4000
    this.pointLight.shadow.mapSize.width = 2048
    this.pointLight.shadow.mapSize.height = 2048

    this.scene!.add(this.ambientLight)
    this.scene!.add(this.pointLight)
  }
  
  initLoadingManager() {
    this.loadingManager = new LoadingManager()
    this.loadingManager.onStart = () => {
      console.log('loading started')
    }
    this.loadingManager.onProgress = (url, loaded, total) => {
      console.log('loading in progress:')
      console.log(`${url} -> ${loaded} / ${total}`)
    }
    this.loadingManager.onLoad = () => {
      console.log('loaded!')
    }
    this.loadingManager.onError = () => {
      console.log('❌ error while loading')
    }
  }
  
  addObjects() {
    const sideLength = 1
    const cubeGeometry = new BoxGeometry(sideLength, sideLength, sideLength)
    const cubeMaterial = new MeshStandardMaterial({
      color: '#f69f1f',
      metalness: 0.5,
      roughness: 0.7,
    })
    this.cube = new Mesh(cubeGeometry, cubeMaterial)
    this.cube.castShadow = true
    this.cube.position.y = 0.5
  
    const planeGeometry = new PlaneGeometry(3, 3)
    const planeMaterial = new MeshLambertMaterial({
      color: 'gray',
      emissive: 'teal',
      emissiveIntensity: 0.2,
      side: 2,
      transparent: true,
      opacity: 0.4,
    })
    const plane = new Mesh(planeGeometry, planeMaterial)
    plane.rotateX(Math.PI / 2)
    plane.receiveShadow = true
  
    this.scene!.add(this.cube)
    this.scene!.add(plane)
  }
  
  addControls() {
    this.dragControls = new DragControls([this.cube!], this.camera!, this.renderer!.domElement)
    this.dragControls.addEventListener('hoveron', (event) => {
      event.object.material.emissive.set('orange')
    })
    this.dragControls.addEventListener('hoveroff', (event) => {
      event.object.material.emissive.set('black')
    })
    this.dragControls.addEventListener('dragstart', (event) => {
      this.cameraControls!.enabled = false
      animation.play = false
      event.object.material.emissive.set('black')
      event.object.material.opacity = 0.7
      event.object.material.needsUpdate = true
    })
    this.dragControls.addEventListener('dragend', (event) => {
      this.cameraControls!.enabled = true
      animation.play = true
      event.object.material.emissive.set('black')
      event.object.material.opacity = 1
      event.object.material.needsUpdate = true
    })
    this.dragControls.enabled = false
  }
  addHelpers() {
    this.axesHelper = new AxesHelper(4)
    this.axesHelper.visible = false
    this.scene!.add(this.axesHelper)

    this.pointLightHelper = new PointLightHelper(this.pointLight!, undefined, 'orange')
    this.pointLightHelper.visible = false
    this.scene!.add(this.pointLightHelper)
  
    const gridHelper = new GridHelper(20, 20, 'teal', 'darkgray')
    gridHelper.position.y = -0.01
    this.scene!.add(gridHelper)
  }
  addGUI() {
    this.gui = new GUI({ title: '🐞 Debug GUI', width: 300 })
  
    if (this.cube) {
      const cubeOneFolder = this.gui.addFolder('Cube one')
      cubeOneFolder.add(this.cube.position, 'x').min(-5).max(5).step(0.5).name('pos x')
      cubeOneFolder.add(this.cube.position, 'y').min(-5).max(5).step(0.5).name('pos y')
      cubeOneFolder.add(this.cube.position, 'z').min(-5).max(5).step(0.5).name('pos z')

      cubeOneFolder.add(this.cube.material, 'wireframe')
      cubeOneFolder.addColor(this.cube.material, 'color')
      cubeOneFolder.add(this.cube.material, 'metalness', 0, 1, 0.1)
      cubeOneFolder.add(this.cube.material, 'roughness', 0, 1, 0.1)
  
      cubeOneFolder.add(this.cube.rotation, 'x', -Math.PI * 2, Math.PI * 2, Math.PI / 4).name('rotate x')
      cubeOneFolder.add(this.cube.rotation, 'y', -Math.PI * 2, Math.PI * 2, Math.PI / 4).name('rotate y')
      cubeOneFolder.add(this.cube.rotation, 'z', -Math.PI * 2, Math.PI * 2, Math.PI / 4).name('rotate z')
  
      cubeOneFolder.add(animation, 'enabled').name('animated')
    }
  
    const controlsFolder = this.gui.addFolder('Controls')
    controlsFolder.add(this.dragControls!, 'enabled').name('drag controls')
  
    const lightsFolder = this.gui.addFolder('Lights')
    lightsFolder.add(this.pointLight!, 'visible').name('point light')
    lightsFolder.add(this.ambientLight!, 'visible').name('ambient light')
  
    const helpersFolder = this.gui.addFolder('Helpers')
    helpersFolder.add(this.axesHelper!, 'visible').name('axes')
    helpersFolder.add(this.pointLightHelper!, 'visible').name('pointLight')
  
    const cameraFolder = this.gui.addFolder('Camera')
    cameraFolder.add(this.cameraControls!, 'autoRotate')
  
    // persist GUI state in local storage on changes
    this.gui.onFinishChange(() => {
      const guiState = this.gui!.save()
      localStorage.setItem('guiState', JSON.stringify(guiState))
    })
  
    // load GUI state if available in local storage
    const guiState = localStorage.getItem('guiState')
    if (guiState) this.gui.load(JSON.parse(guiState))
  
    // reset GUI state button
    const resetGui = () => {
      localStorage.removeItem('guiState')
      this.gui!.reset()
    }
    this.gui.add({ resetGui }, 'resetGui').name('RESET')
  
    this.gui.close()
  }
  addClock() {
    this.clock = new Clock()
  }
  addStats() {
    this.stats = new Stats()
    document.body.appendChild(this.stats.dom)
  }
  
  addBloomComposer() {
    // bloom效果
    const renderScene = new RenderPass(this.scene!, this.camera!);
    const bloomPass = new UnrealBloomPass(
      new Vector2(window.innerWidth, window.innerHeight),
      1.5,
      0.4,
      0.85
    );
    bloomPass.threshold = 0.3;
    bloomPass.strength = 2;
    bloomPass.radius = 0.3;
    this.bloomComposer = new EffectComposer(this.renderer!);
    this.bloomComposer.addPass(renderScene);
    this.bloomComposer.addPass(bloomPass);
  }
}

new Main()
