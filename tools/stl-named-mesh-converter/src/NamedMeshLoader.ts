import type { Group } from "three";
import { loadStlAsync, parseStlToGroup } from "./stlLoader.js";
import { loadStepAsync, parseStepToGroup } from "./stepLoader.js";
import type { NamedMeshLoaderOptions } from "./types.js";

const STL_EXT = ".stl";
const STEP_EXT = ".step";
const STP_EXT = ".stp";

function extensionFromUrl(url: string): string {
  const q = url.indexOf("?");
  const path = q >= 0 ? url.slice(0, q) : url;
  const last = path.lastIndexOf(".");
  if (last < 0) return "";
  return path.slice(last).toLowerCase();
}

/**
 * Unified loader for STL and STEP files. Returns a Three.js Group whose
 * children are named meshes, so you can animate by name, e.g.
 * group.getObjectByName('Arm').
 */
export class NamedMeshLoader {
  private options?: NamedMeshLoaderOptions;

  constructor(options?: NamedMeshLoaderOptions) {
    this.options = options;
  }

  /**
   * Load from URL. Dispatches by file extension (.stl, .stp, .step).
   */
  async loadAsync(url: string): Promise<Group> {
    const ext = extensionFromUrl(url);
    if (ext === STL_EXT) {
      return loadStlAsync(url, this.options);
    }
    if (ext === STEP_EXT || ext === STP_EXT) {
      return loadStepAsync(url, this.options);
    }
    throw new Error(
      `Unsupported file extension: ${ext || "(none)"}. Use .stl, .stp, or .step.`
    );
  }

  /**
   * Parse from buffer with explicit extension (e.g. '.stl' or '.stp').
   */
  async parseAsync(data: ArrayBuffer, extension: string): Promise<Group> {
    const ext = extension.toLowerCase();
    if (ext === STL_EXT) {
      return Promise.resolve(parseStlToGroup(data, this.options));
    }
    if (ext === STEP_EXT || ext === STP_EXT) {
      return parseStepToGroup(data, this.options);
    }
    throw new Error(
      `Unsupported extension: ${extension}. Use .stl, .stp, or .step.`
    );
  }
}
