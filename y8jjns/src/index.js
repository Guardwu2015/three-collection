import "./styles.css";
import * as THREE from "three";
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls'

// Setup
const scene = new THREE.Scene();
const renderer = new THREE.WebGLRenderer({
  canvas: document.querySelector("#three")
});
renderer.setSize(window.innerWidth, window.innerHeight);

window.addEventListener("resize", () => {
  renderer.setSize(window.innerWidth, window.innerHeight);
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  uniforms.u_resolution.value.x = renderer.domElement.width;
  uniforms.u_resolution.value.y = renderer.domElement.height;
});

window.addEventListener("mousemove", (e) => {
  uniforms.u_mouse.value.x = e.pageX;
  uniforms.u_mouse.value.y = e.pageY;
});

// Camera
// const camera = new THREE.OrthographicCamera(-1, 1, 1, -1, 1, 1000);
const camera = new THREE.PerspectiveCamera(
  45,
  window.innerWidth / window.innerHeight,
  1,
  1000
);
camera.position.z = 4;

const controls = new OrbitControls(camera, renderer.domElement);

// Main Code
const uniforms = {
  u_time: { value: 1.0 },
  u_resolution: { value: new THREE.Vector2() },
  u_mouse: { value: new THREE.Vector2() }
};

const geometry = new THREE.SphereGeometry(1, 200, 200);
const material = new THREE.ShaderMaterial({
  uniforms,
  vertexShader: document.getElementById("vertexShader").textContent,
  fragmentShader: document.getElementById("fragmentShader").textContent
});
const cube = new THREE.Mesh(geometry, material);
scene.add(cube);

const clock = new THREE.Clock();
function animate() {
  window.requestAnimationFrame(animate);
  controls.update();
  uniforms.u_time.value += clock.getDelta();
  renderer.render(scene, camera);
}
animate();
