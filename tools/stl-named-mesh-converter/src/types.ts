import type { Group, Material } from "three";

/**
 * Options for loading STL or STEP and building a Group of named meshes.
 */
export interface NamedMeshLoaderOptions {
  /**
   * Default material used for meshes when none is provided per-part.
   * @default MeshStandardMaterial({ color: 0xcccccc })
   */
  defaultMaterial?: Material;
  /**
   * Name for the single mesh when the file has only one part (binary STL or single solid).
   * @default 'mesh'
   */
  defaultMeshName?: string;
  /**
   * Base path for resolving relative URLs (STLLoader).
   */
  basePath?: string;
}

/**
 * Result of loading an STL or STEP file: a Three.js Group whose direct children
 * are named meshes, so you can animate by name, e.g. group.getObjectByName('Arm').
 */
export type NamedMeshLoadResult = Group;
