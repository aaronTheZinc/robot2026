import {
  BufferGeometry,
  Group,
  Mesh,
  MeshStandardMaterial,
  type Material,
} from "three";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import type { NamedMeshLoaderOptions } from "./types.js";

const DEFAULT_MATERIAL = new MeshStandardMaterial({ color: 0xcccccc });

function getDefaultMaterial(options?: NamedMeshLoaderOptions): Material {
  return options?.defaultMaterial ?? DEFAULT_MATERIAL;
}

function getDefaultMeshName(options?: NamedMeshLoaderOptions): string {
  return options?.defaultMeshName ?? "mesh";
}

/**
 * Parse STL data (ArrayBuffer) and return a Group of named meshes.
 * - ASCII with multiple solids: one Mesh per solid, name from solid block or 'solid_N'.
 * - ASCII single solid / Binary: one Mesh with defaultMeshName.
 */
export function parseStlToGroup(
  data: ArrayBuffer,
  options?: NamedMeshLoaderOptions
): Group {
  const loader = new STLLoader();
  if (options?.basePath) {
    loader.setPath(options.basePath);
  }
  const geometry = loader.parse(data);
  return geometryToNamedGroup(geometry, options);
}

/**
 * Load STL from URL and return a Group of named meshes.
 */
export function loadStlAsync(
  url: string,
  options?: NamedMeshLoaderOptions
): Promise<Group> {
  const loader = new STLLoader();
  if (options?.basePath) {
    loader.setPath(options.basePath);
  }
  return loader.loadAsync(url).then((geometry) => {
    return geometryToNamedGroup(geometry, options);
  });
}

function geometryToNamedGroup(
  geometry: BufferGeometry & { userData?: { groupNames?: string[] }; groups?: Array<{ start: number; count: number }> },
  options?: NamedMeshLoaderOptions
): Group {
  const material = getDefaultMaterial(options);
  const defaultName = getDefaultMeshName(options);
  const group = new Group();

  const groups = geometry.groups;
  const groupNames =
    (geometry.userData?.groupNames as string[] | undefined) ?? [];

  if (groups.length > 0 && groupNames.length >= groups.length) {
    // Multiple named solids (ASCII STL)
    for (let i = 0; i < groups.length; i++) {
      const g = groups[i];
      const name = (groupNames[i] ?? "").trim() || `solid_${i}`;
      const mesh = createMeshForGroup(geometry, g.start, g.count, material);
      mesh.name = name;
      group.add(mesh);
    }
  } else {
    // Single mesh (binary STL or ASCII single solid)
    const mesh = new Mesh(geometry, material);
    mesh.name = defaultName;
    group.add(mesh);
  }

  return group;
}

/**
 * Create a Mesh that draws only the vertex range [start, start+count).
 * Clones the geometry and sets drawRange so we don't duplicate buffers.
 */
function createMeshForGroup(
  geometry: BufferGeometry,
  start: number,
  count: number,
  material: Material
): Mesh {
  const clone = geometry.clone();
  clone.setDrawRange(start, count);
  clone.groups.length = 0;
  return new Mesh(clone, material);
}
