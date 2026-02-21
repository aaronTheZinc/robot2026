/**
 * STEP (.stp / .step) loader using opencascade.js (optional dependency).
 * If opencascade.js is not installed, loadStepAsync/parseStepToGroup throw a clear error.
 */

import {
  BufferAttribute,
  BufferGeometry,
  Group,
  Mesh,
  MeshStandardMaterial,
  type Material,
} from "three";
import type { NamedMeshLoaderOptions } from "./types.js";

const STEP_NOT_INSTALLED_MSG =
  "STEP support requires the optional dependency 'opencascade.js'. Install it with: npm install opencascade.js";

type OCApi = {
  default: () => Promise<unknown>;
};

function getDefaultMaterial(options?: NamedMeshLoaderOptions): Material {
  return options?.defaultMaterial ?? new MeshStandardMaterial({ color: 0xcccccc });
}

function getDefaultMeshName(options?: NamedMeshLoaderOptions): string {
  return options?.defaultMeshName ?? "mesh";
}

/**
 * Load STEP from URL: fetch the file then parse with parseStepToGroup.
 * Requires opencascade.js to be installed.
 */
export async function loadStepAsync(
  url: string,
  options?: NamedMeshLoaderOptions
): Promise<Group> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`Failed to fetch STEP: ${res.status} ${res.statusText}`);
  const buffer = await res.arrayBuffer();
  return parseStepToGroup(buffer, options);
}

/**
 * Parse STEP data (ArrayBuffer) and return a Group of named meshes.
 * Each top-level shape becomes one mesh (part_0, part_1, ... or defaultMeshName if single).
 * Requires opencascade.js to be installed.
 */
export async function parseStepToGroup(
  data: ArrayBuffer,
  options?: NamedMeshLoaderOptions
): Promise<Group> {
  let init: (() => Promise<unknown>) | undefined;
  try {
    const mod = await import("opencascade.js") as OCApi;
    init = mod.default;
  } catch {
    throw new Error(STEP_NOT_INSTALLED_MSG);
  }
  if (!init) throw new Error(STEP_NOT_INSTALLED_MSG);

  const oc = (await init()) as {
    FS: { writeFile: (path: string, data: Uint8Array | ArrayBufferView) => void; unlink?: (path: string) => void };
    STEPControl_Reader: new () => {
      ReadFile: (path: string) => number;
      TransferRoots: (progress: unknown) => void;
      NbShapes: () => number;
      Shape: (index: number) => unknown;
    };
    BRepMesh_IncrementalMesh: new (shape: unknown, linearDeflection: number, relative: boolean, angularDeflection: number, parallel: boolean) => void;
    TopExp_Explorer: new (shape: unknown, shapeType: number) => {
      More: () => boolean;
      Next: () => void;
      Current: () => unknown;
    };
    TopAbs_ShapeEnum: { TopAbs_FACE: number };
    BRep_Tool: {
      Triangulation: (face: unknown, loc: unknown, purpose?: number) => unknown;
    };
    Poly_MeshPurpose: { Poly_MeshPurpose_None: number };
    Poly_Triangulation: new () => {
      NbNodes: () => number;
      NbTriangles: () => number;
      Node: (i: number) => { X: () => number; Y: () => number; Z: () => number };
      Triangle: (i: number) => { Value: (i: number) => number };
    };
    TopLoc_Location: new () => unknown;
  };

  const path = "/tmp_step_input.stp";
  oc.FS.writeFile(path, new Uint8Array(data));

  const reader = new oc.STEPControl_Reader();
  const readStatus = reader.ReadFile(path);
  if (typeof readStatus !== "number" || readStatus !== 1) {
    if (oc.FS.unlink) oc.FS.unlink(path);
    throw new Error("STEP read failed: invalid file or format");
  }

  try {
    const ProgressRange = (oc as unknown as { Message_ProgressRange_1?: new () => unknown }).Message_ProgressRange_1;
    if (ProgressRange) reader.TransferRoots(new ProgressRange());
    else reader.TransferRoots(undefined as unknown as never);
  } catch {
    reader.TransferRoots(undefined as unknown as never);
  }

  const material = getDefaultMaterial(options);
  const defaultName = getDefaultMeshName(options);
  const group = new Group();

  const nbShapes = reader.NbShapes();
  const TopAbs_FACE = oc.TopAbs_ShapeEnum.TopAbs_FACE;
  const meshPurpose = (oc.Poly_MeshPurpose?.Poly_MeshPurpose_None ?? 0) as number;

  for (let s = 1; s <= nbShapes; s++) {
    const shape = reader.Shape(s);
    if (!shape) continue;

    new oc.BRepMesh_IncrementalMesh(shape, 0.1, false, 0.5, false);

    const vertices: number[] = [];
    const normals: number[] = [];

    const exp = new oc.TopExp_Explorer(shape, TopAbs_FACE);
    while (exp.More()) {
      const face = exp.Current();
      const loc = new oc.TopLoc_Location();
      const handle = oc.BRep_Tool.Triangulation(face, loc, meshPurpose) as { get?: () => unknown } | null;
      if (!handle || typeof (handle as { get?: () => unknown }).get !== "function") continue;
      const tri = (handle as { get: () => unknown }).get() as {
        NbNodes: () => number;
        NbTriangles: () => number;
        Node: (i: number) => { X: () => number; Y: () => number; Z: () => number };
        Triangle: (i: number) => { Value: (i: number) => number };
      };
      if (!tri || typeof tri.NbNodes !== "function") continue;

      const nNodes = tri.NbNodes();
      const nTris = tri.NbTriangles();
      const base = vertices.length / 3;

      for (let i = 1; i <= nNodes; i++) {
        const n = tri.Node(i);
        vertices.push(n.X(), n.Y(), n.Z());
        normals.push(0, 0, 0);
      }

      for (let i = 1; i <= nTris; i++) {
        const t = tri.Triangle(i);
        const i1 = (t.Value(1) ?? 1) - 1;
        const i2 = (t.Value(2) ?? 2) - 1;
        const i3 = (t.Value(3) ?? 3) - 1;
        const a = base + i1;
        const b = base + i2;
        const c = base + i3;
        const ax = vertices[a * 3], ay = vertices[a * 3 + 1], az = vertices[a * 3 + 2];
        const bx = vertices[b * 3], by = vertices[b * 3 + 1], bz = vertices[b * 3 + 2];
        const cx = vertices[c * 3], cy = vertices[c * 3 + 1], cz = vertices[c * 3 + 2];
        const nx = (by - ay) * (cz - az) - (bz - az) * (cy - ay);
        const ny = (bz - az) * (cx - ax) - (bx - ax) * (cz - az);
        const nz = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        const len = Math.sqrt(nx * nx + ny * ny + nz * nz) || 1;
        normals[a * 3] += nx / len; normals[a * 3 + 1] += ny / len; normals[a * 3 + 2] += nz / len;
        normals[b * 3] += nx / len; normals[b * 3 + 1] += ny / len; normals[b * 3 + 2] += nz / len;
        normals[c * 3] += nx / len; normals[c * 3 + 1] += ny / len; normals[c * 3 + 2] += nz / len;
      }

      exp.Next();
    }

    if (vertices.length === 0) continue;

    for (let i = 0; i < normals.length; i += 3) {
      const len = Math.sqrt(normals[i] ** 2 + normals[i + 1] ** 2 + normals[i + 2] ** 2) || 1;
      normals[i] /= len; normals[i + 1] /= len; normals[i + 2] /= len;
    }

    const geometry = new BufferGeometry();
    geometry.setAttribute("position", new BufferAttribute(new Float32Array(vertices), 3));
    geometry.setAttribute("normal", new BufferAttribute(new Float32Array(normals), 3));

    const mesh = new Mesh(geometry, material);
    mesh.name = nbShapes === 1 ? defaultName : `part_${s - 1}`;
    group.add(mesh);
  }

  if (oc.FS.unlink) oc.FS.unlink(path);

  if (group.children.length === 0) {
    throw new Error("STEP file produced no geometry");
  }

  return group;
}
