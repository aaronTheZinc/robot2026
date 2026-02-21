# STL / STEP to Named Mesh Converter

Load STL and STEP (`.stp` / `.step`) files as a Three.js `Group` whose children are **named meshes**, so you can animate specific parts by name (e.g. `group.getObjectByName('Arm')`).

## Install

```bash
npm install three
npm install stl-named-mesh-converter
```

For **STEP** support (optional):

```bash
npm install opencascade.js
```

Without `opencascade.js`, only STL files can be loaded; loading a `.stp` or `.step` file will throw a clear error.

## Usage

```js
import * as THREE from "three";
import { NamedMeshLoader } from "stl-named-mesh-converter";

const loader = new NamedMeshLoader();
const group = await loader.loadAsync("/models/robot.stl");  // or .stp / .step
scene.add(group);

// Animate a specific mesh by name
const arm = group.getObjectByName("Arm");
if (arm) {
  arm.rotation.x += 0.01;
}
```

### From a buffer

```js
const buffer = await response.arrayBuffer();
const group = await loader.parseAsync(buffer, ".stl");
scene.add(group);
```

### Options

```js
const loader = new NamedMeshLoader({
  defaultMaterial: new THREE.MeshStandardMaterial({ color: 0x888888 }),
  defaultMeshName: "model",  // name when file has a single part
  basePath: "/models",       // base path for STL relative URLs
});
```

## Animation

The loader returns a standard Three.js `Group`. Each child is a `Mesh` with a stable `name`. Use `getObjectByName()` or traverse to drive animations:

```js
// In your render loop or animation frame
const part = group.getObjectByName("Turret");
if (part) {
  part.rotation.y += 0.02;
}
```

You can use Three.js animation utilities, keyframes, or libraries like Tween.js on these objects as usual.

## STL vs STEP

| Format | Names | Notes |
|--------|--------|--------|
| **STL (ASCII)** | Multiple `solid name ... endsolid` blocks become separate named meshes. | For multiple animatable parts, export ASCII STL with multiple solids. |
| **STL (binary)** | Single mesh; name is `defaultMeshName` (default `"mesh"`). | No per-part names. |
| **STEP / STP** | One mesh per top-level shape: `part_0`, `part_1`, … or `defaultMeshName` if only one shape. | Requires optional dependency `opencascade.js`. Product/label names from the file may be used in a future version. |

## API

- **`NamedMeshLoader`** – Unified loader. `loadAsync(url)` and `parseAsync(buffer, extension)` return `Promise<THREE.Group>`.
- **`loadStlAsync(url, options?)`** / **`parseStlToGroup(buffer, options?)`** – STL only.
- **`loadStepAsync(url, options?)`** / **`parseStepToGroup(buffer, options?)`** – STEP only (requires `opencascade.js`).

## Integration

From another app in this repo (e.g. `robot-dashboard`), add the package as a workspace dependency or install from the `tools/stl-named-mesh-converter` path, and ensure `three` is installed. STEP support is optional; install `opencascade.js` only if you need `.stp` / `.step` files.
