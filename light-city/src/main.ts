import GUI from 'lil-gui'
import {
  AmbientLight,
  AnimationMixer,
  AxesHelper,
  BackSide,
  BoxGeometry,
  Clock,
  DoubleSide,
  EdgesGeometry,
  Float32BufferAttribute,
  FrontSide,
  GridHelper,
  Group,
  LineBasicMaterial,
  LineSegments,
  LoadingManager,
  Material,
  Mesh,
  MeshBasicMaterial,
  MeshLambertMaterial,
  MeshStandardMaterial,
  Object3D,
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
import { interpolateHsl } from "d3-interpolate";
import TWEEN from "@tweenjs/tween.js";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js"
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

let canvas: HTMLElement
let renderer: WebGLRenderer
let scene: Scene
let loadingManager: LoadingManager
let ambientLight: AmbientLight
let pointLight: PointLight
let cube: Mesh
let camera: PerspectiveCamera
let cameraControls: OrbitControls
let dragControls: DragControls
let axesHelper: AxesHelper
let pointLightHelper: PointLightHelper
let clock: Clock
let stats: Stats
let gui: GUI
let bloomComposer: EffectComposer

const mixers: AnimationMixer[] = []

const animation = { enabled: false, play: true }

init()

function init() {
  initScene()
  initRenderer()
  initCamera()
  initLights()
  initLoadingManager()

  // addObjects()
  loadCity()
  addControls()
  addHelpers()
  addGUI()
  addClock()
  addStats()
  // addBloomComposer()

  // Full screen
  window.addEventListener('dblclick', (event) => {
    if (event.target === canvas) {
      toggleFullScreen(canvas)
    }
  })

  animate()
}

function animate() {
  requestAnimationFrame(animate)

  stats.update()

  if (animation.enabled && animation.play) {
    animations.rotate(cube, clock, Math.PI / 3)
    animations.bounce(cube, clock, 1, 0.5, 0.5)
  }

  if (resizeRendererToDisplaySize(renderer)) {
    const canvas = renderer.domElement
    camera.aspect = canvas.clientWidth / canvas.clientHeight
    camera.updateProjectionMatrix()
  }

  cameraControls.update()

  TWEEN.update();

  if (mixers.length) {
    const delta = clock.getDelta();
    for (const mixer of mixers) {
      mixer.update(delta);
    }
  }

  renderer.render(scene, camera)
  // bloomComposer.render()
}

function initScene() {
  scene = new Scene()
}

function initRenderer() {
  canvas = document.querySelector(`canvas#${CANVAS_ID}`)!
  renderer = new WebGLRenderer({ canvas, antialias: true, alpha: true })
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
  renderer.shadowMap.enabled = true
  renderer.shadowMap.type = PCFSoftShadowMap
}

function initCamera() {
  camera = new PerspectiveCamera(50, canvas.clientWidth / canvas.clientHeight, 0.1, 100)
  camera.position.set(2, 2, 5)

  cameraControls = new OrbitControls(camera, canvas)
  cameraControls.target = new Vector3(0, 0, 0)
  cameraControls.enableDamping = true
  cameraControls.autoRotate = false
  cameraControls.update()
}

function initLights() {
  ambientLight = new AmbientLight('white', 0.4)
  pointLight = new PointLight('#ffdca8', 1.2, 100)
  pointLight.position.set(-2, 3, 3)
  pointLight.castShadow = true
  pointLight.shadow.radius = 4
  pointLight.shadow.camera.near = 0.5
  pointLight.shadow.camera.far = 4000
  pointLight.shadow.mapSize.width = 2048
  pointLight.shadow.mapSize.height = 2048
  scene.add(ambientLight)
  scene.add(pointLight)
}

function initLoadingManager() {
  loadingManager = new LoadingManager()
  loadingManager.onStart = () => {
    console.log('loading started')
  }
  loadingManager.onProgress = (url, loaded, total) => {
    console.log('loading in progress:')
    console.log(`${url} -> ${loaded} / ${total}`)
  }
  loadingManager.onLoad = () => {
    console.log('loaded!')
  }
  loadingManager.onError = () => {
    console.log('❌ error while loading')
  }
}

function addObjects() {
  const sideLength = 1
  const cubeGeometry = new BoxGeometry(sideLength, sideLength, sideLength)
  const cubeMaterial = new MeshStandardMaterial({
    color: '#f69f1f',
    metalness: 0.5,
    roughness: 0.7,
  })
  cube = new Mesh(cubeGeometry, cubeMaterial)
  cube.castShadow = true
  cube.position.y = 0.5

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

  scene.add(cube)
  scene.add(plane)
}

async function loadCity() {
  const loader = new GLTFLoader()
  const obj = await loader.loadAsync(
    "models/b-city.glb"
  );

  scene.add(obj.scene)

  obj?.scene?.children.forEach((item) => {
    change2BasicMat(item);
  });

  const roadNum = 4;
  for (let i = 1; i <= roadNum; i++) {
    const name = `road${i ? "00" + i : ""}`;
    const road = scene.getObjectByName(name);
    change2LightTrail(road);
  }

  const elevatorList = ["Elevator", "Elevator001"];
  elevatorList.forEach((item, index) => {
    const elevator = scene.getObjectByName(item);
    const mesh = change2LightBox(elevator);
    // 创建Animation
    const mixer = new AnimationMixer(elevator);
    mixer.clipAction(obj.animations[index]).play();
    mixers.push(mixer);
  });
}

const basicMat = new MeshBasicMaterial({
  opacity: 0.25,
  color: 0x1f56b9,
  side: BackSide,
  transparent: true,
});

function change2BasicMat(obj) {
  obj.traverse(item => {
    if (item.material) {
      item.material = basicMat
    }
  });
}

function change2LightTrail(object3d) {
  // 使用顶点颜色 VertexColors
  const material = new MeshBasicMaterial({
    vertexColors: true,
    side: BackSide,
  });
  const geometry = object3d.geometry.clone();
  // 生成渐变色的color数组
  const count = geometry.attributes.position.count;
  const rgbInterpolate = interpolateHsl("#00ffff", "#000000");
  const colorArray = new Array(count);
  for (let index = 0; index < count; index++) {
    const t = index / count;
    const rgb = rgbInterpolate(t);
    const rgbValue = rgb.match(/\d+/g);
    // 从 "rgb(1,2,3)" 字符串里 提取出 1,2,3 并 归一化（ 0.0 ~ 1.0）
    const r = Number(rgbValue[0]) / 255;
    const g = Number(rgbValue[1]) / 255;
    const b = Number(rgbValue[2]) / 255;

    colorArray[3 * index] = r;
    colorArray[3 * index + 1] = g;
    colorArray[3 * index + 2] = b;
  }

  const anchor = Number((Math.random() * count).toFixed(0));
  const b = colorArray.slice(anchor * 3);
  const f = colorArray.slice(0, anchor * 3);
  const newColorArray = [].concat(b, f);

  geometry.setAttribute(
    "color",
    new Float32BufferAttribute(newColorArray, 3)
  );
  const mesh = new Mesh(geometry, material);
  mesh.position.set(
    object3d.position.x,
    object3d.position.y,
    object3d.position.z
  );
  mesh.rotation.set(
    object3d.rotation.x,
    object3d.rotation.y,
    object3d.rotation.z
  );
  mesh.scale.set(object3d.scale.x, object3d.scale.y, object3d.scale.z);
  object3d.parent.add(mesh);
  setInterval(() => {
    lightMove(mesh, newColorArray);
  }, 2000);
}

function change2LightBox(object3d) {
  const group = new Group();
  const lineMaterial = new LineBasicMaterial({ color: 0x00ffff });
  const boxMaterial = new MeshBasicMaterial({
    opacity: 0.5,
    color: 0x00cccc,
    side: DoubleSide,
    transparent: true,
  });

  object3d.material = boxMaterial
  const geo = new EdgesGeometry(object3d.geometry);
  const line = new LineSegments(geo, lineMaterial);
  const box = new Mesh(object3d.geometry, boxMaterial);

  group.add(line).add(box);

  group.position.copy(object3d.position)
  group.rotation.copy(object3d.rotation)
  group.scale.copy(object3d.scale)

  // object3d.parent.add(group);
  // object3d.visible = false;

  return group;
}

// 颜色变化
function lightMove(mesh, colorArray) {
  const len = colorArray.length / 3;
  new TWEEN.Tween({ value: 0 })
    .to({ value: 1 }, 2000)
    .onUpdate(function (val) {
      // 实现环状数组变化
      const anchor = Number((val.value * len).toFixed(0));
      const b = colorArray.slice(anchor * 3);
      const f = colorArray.slice(0, anchor * 3);
      const newColorArray = [].concat(b, f);
      mesh.geometry.setAttribute(
        "color",
        new Float32BufferAttribute(newColorArray, 3)
      );
    })
    .start();
}

function addControls() {
  dragControls = new DragControls([cube], camera, renderer.domElement)
  dragControls.addEventListener('hoveron', (event) => {
    event.object.material.emissive.set('orange')
  })
  dragControls.addEventListener('hoveroff', (event) => {
    event.object.material.emissive.set('black')
  })
  dragControls.addEventListener('dragstart', (event) => {
    cameraControls.enabled = false
    animation.play = false
    event.object.material.emissive.set('black')
    event.object.material.opacity = 0.7
    event.object.material.needsUpdate = true
  })
  dragControls.addEventListener('dragend', (event) => {
    cameraControls.enabled = true
    animation.play = true
    event.object.material.emissive.set('black')
    event.object.material.opacity = 1
    event.object.material.needsUpdate = true
  })
  dragControls.enabled = false
}
function addHelpers() {
  axesHelper = new AxesHelper(4)
  axesHelper.visible = false
  scene.add(axesHelper)

  pointLightHelper = new PointLightHelper(pointLight, undefined, 'orange')
  pointLightHelper.visible = false
  scene.add(pointLightHelper)

  // const gridHelper = new GridHelper(20, 20, 'teal', 'darkgray')
  // gridHelper.position.y = -0.01
  // scene.add(gridHelper)
}
function addGUI() {
  gui = new GUI({ title: '🐞 Debug GUI', width: 300 })

  if (cube) {
    const cubeOneFolder = gui.addFolder('Cube one')
    cubeOneFolder.add(cube.position, 'x').min(-5).max(5).step(0.5).name('pos x')
    cubeOneFolder.add(cube.position, 'y').min(-5).max(5).step(0.5).name('pos y')
    cubeOneFolder.add(cube.position, 'z').min(-5).max(5).step(0.5).name('pos z')

    cubeOneFolder.add(cube.material, 'wireframe')
    cubeOneFolder.addColor(cube.material, 'color')
    cubeOneFolder.add(cube.material, 'metalness', 0, 1, 0.1)
    cubeOneFolder.add(cube.material, 'roughness', 0, 1, 0.1)

    cubeOneFolder.add(cube.rotation, 'x', -Math.PI * 2, Math.PI * 2, Math.PI / 4).name('rotate x')
    cubeOneFolder.add(cube.rotation, 'y', -Math.PI * 2, Math.PI * 2, Math.PI / 4).name('rotate y')
    cubeOneFolder.add(cube.rotation, 'z', -Math.PI * 2, Math.PI * 2, Math.PI / 4).name('rotate z')

    cubeOneFolder.add(animation, 'enabled').name('animated')
  }

  const controlsFolder = gui.addFolder('Controls')
  controlsFolder.add(dragControls, 'enabled').name('drag controls')

  const lightsFolder = gui.addFolder('Lights')
  lightsFolder.add(pointLight, 'visible').name('point light')
  lightsFolder.add(ambientLight, 'visible').name('ambient light')

  const helpersFolder = gui.addFolder('Helpers')
  helpersFolder.add(axesHelper, 'visible').name('axes')
  helpersFolder.add(pointLightHelper, 'visible').name('pointLight')

  const cameraFolder = gui.addFolder('Camera')
  cameraFolder.add(cameraControls, 'autoRotate')

  // persist GUI state in local storage on changes
  gui.onFinishChange(() => {
    const guiState = gui.save()
    localStorage.setItem('guiState', JSON.stringify(guiState))
  })

  // load GUI state if available in local storage
  const guiState = localStorage.getItem('guiState')
  if (guiState) gui.load(JSON.parse(guiState))

  // reset GUI state button
  const resetGui = () => {
    localStorage.removeItem('guiState')
    gui.reset()
  }
  gui.add({ resetGui }, 'resetGui').name('RESET')

  gui.close()
}
function addClock() {
  clock = new Clock()
}
function addStats() {
  stats = new Stats()
  document.body.appendChild(stats.dom)
}

function addBloomComposer() {
  // bloom效果
  const renderScene = new RenderPass(scene, camera);
  const bloomPass = new UnrealBloomPass(
    new Vector2(window.innerWidth, window.innerHeight),
    1.5,
    0.4,
    0.85
  );
  bloomPass.threshold = 0.3;
  bloomPass.strength = 2;
  bloomPass.radius = 0.3;
  bloomComposer = new EffectComposer(renderer);
  bloomComposer.addPass(renderScene);
  bloomComposer.addPass(bloomPass);
}
