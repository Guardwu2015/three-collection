import * as $ from "//unpkg.com/three@0.123.0/build/three.module.js";
import { OrbitControls } from "//unpkg.com/three@0.123.0/examples/jsm/controls/OrbitControls.js";

const renderer = new $.WebGLRenderer({ antialias: true });
const scene = new $.Scene();
const camera = new $.PerspectiveCamera(75, 2, 0.1, 1000);
const controls = new OrbitControls(camera, renderer.domElement);
window.addEventListener("resize", () => {
  const { clientWidth, clientHeight } = renderer.domElement;
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(clientWidth, clientHeight, false);
  camera.aspect = clientWidth / clientHeight;
  camera.updateProjectionMatrix();
});
document.body.prepend(renderer.domElement);
window.dispatchEvent(new Event("resize"));
renderer.setAnimationLoop((t) => {
  renderer.render(scene, camera);
  controls.update();
  animate(t);
});

// ----
// Main
// ----
$.ShaderChunk.my_map_fragment = `
#ifdef USE_MAP
    float t = t * 0.0007;
    vec2 uv = vUv * textureScale + vec2(-1.0, 0.5); // scale offset old uv
    vec4 A = texture2D(map, uv);           // old texel 

    float strength = 130.9; // controls the swirling strength
    float rotationRadius = 0.5; // distance from the center where the rotation happens
    vec2 centeredUv = uv - vec2(0.5, 0.5); // shifting UVs for the swirl effect
    float distanceFromCenter = length(centeredUv);

    // Swirling Effect
    if (distanceFromCenter < rotationRadius) {
        float angle = strength * (rotationRadius - distanceFromCenter) / rotationRadius; // the closer to the center, the stronger the rotation
        float s = sin(angle);
        float c = cos(angle);
        uv = vec2(dot(centeredUv, vec2(c, -s)), dot(centeredUv, vec2(s, c))) + vec2(0.5, 0.5);
    }

    // Your Original Animation
    uv.x += sin(uv.y + t) * 0.01;
    uv.y += length(A.rgb) / 30.1 - t;

    vec4 B = texture2D(map, uv);           // new texel

    // Define the blue color (adjust the values as needed)
    vec3 blueColor = vec3(0.0, 0.0, 1.0);

    // Define a threshold for color difference (you can adjust this value)
    float colorThreshold = 0.5;

    // Calculate the color difference between the current texel and blue color
    float colorDifference = length(B.rgb - blueColor);

    // Set alpha to 1.0 if the color is not close to the blue color, otherwise, set it to 0.0
    float alpha = 1.0 - step(colorThreshold, colorDifference);

    vec4 texelColor = vec4(B.rgb, alpha);
    
    texelColor = mapTexelToLinear(texelColor);
    diffuseColor = texelColor;
#endif
`;

// https://unsplash.com/photos/QSXOERX45BI
const IMGURL =
  "https://assets.codepen.io/9234665/E4484616-BDB4-4AB8-8A83-84A50CF12C79_1_201_a.jpeg";
const RATIO = 62 / 901;

scene.background = new $.Color("white");

const light0 = new $.DirectionalLight("white", 0.9);
light0.position.set(0, 0, 1);
scene.add(light0);

const tex = new $.TextureLoader().load(IMGURL);
tex.wrapS = tex.wrapT = $.MirroredRepeatWrapping;
const geom = new $.SphereBufferGeometry(500, 32, 32); // Create a sphere geometry with a radius of 500
const mat = new $.ShaderMaterial({
  uniforms: {
    ...$.ShaderLib.phong.uniforms,
    t: { value: 0 },
    textureScale: { value: 2.9 }, // Initial zoom value
  },
  vertexShader: $.ShaderLib.phong.vertexShader,
  fragmentShader:
    `
    uniform float t;
    uniform float textureScale;
    ` +
    $.ShaderLib.phong.fragmentShader.replace(
      "#include <map_fragment>",
      "#include <my_map_fragment>"
    ),
  lights: true,
  side: $.BackSide,
});

mat.map = mat.uniforms.map.value = tex;

const sphereMesh = new $.Mesh(geom, mat);
sphereMesh.scale.x = -1; // Invert the x-axis to render on the inside of the sphere
scene.add(sphereMesh);
sphereMesh.rotation.y = -1.5;
sphereMesh.rotation.x = -0.5;
camera.position.set(0, 0, 0);

//// Anim

function animate(t /*sec*/) {
  mat.uniforms.t.value = t * 0.01;
  sphereMesh.rotation.x -= 0.0001;
  sphereMesh.rotation.z += 0.000001;
  mat.uniforms.textureScale.value += 0.0000001;
}
