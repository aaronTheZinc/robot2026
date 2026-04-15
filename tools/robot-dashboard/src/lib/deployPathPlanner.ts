import type { PathPlannerPathJson } from './pathPlannerBezier';

/** Vite: raw path strings from repo deploy folder (relative to this file). */
const pathGlob = import.meta.glob('../../../../bot/src/main/deploy/pathplanner/paths/*.path', {
  query: '?raw',
  import: 'default',
  eager: true,
}) as Record<string, string>;

const autoGlob = import.meta.glob('../../../../bot/src/main/deploy/pathplanner/autos/*.auto', {
  query: '?raw',
  import: 'default',
  eager: true,
}) as Record<string, string>;

function basenameFromGlobKey(key: string, suffix: string): string {
  const slash = key.lastIndexOf('/');
  const file = slash >= 0 ? key.slice(slash + 1) : key;
  return file.endsWith(suffix) ? file.slice(0, -suffix.length) : file.replace(/\.[^.]+$/, '');
}

export function listDeployPathNames(): string[] {
  return Object.keys(pathGlob)
    .map((k) => basenameFromGlobKey(k, '.path'))
    .sort((a, b) => a.localeCompare(b));
}

export function getDeployPathRaw(name: string): string | null {
  const entry = Object.entries(pathGlob).find(
    ([key]) => basenameFromGlobKey(key, '.path') === name
  );
  return entry ? entry[1] : null;
}

export function parsePathJson(raw: string): PathPlannerPathJson | null {
  try {
    const o = JSON.parse(raw) as PathPlannerPathJson;
    if (!o || !Array.isArray(o.waypoints)) {
      return null;
    }
    return o;
  } catch {
    return null;
  }
}

export function listDeployAutoNames(): string[] {
  return Object.keys(autoGlob)
    .map((k) => basenameFromGlobKey(k, '.auto'))
    .sort((a, b) => a.localeCompare(b));
}

export function getDeployAutoRaw(name: string): string | null {
  const entry = Object.entries(autoGlob).find(
    ([key]) => basenameFromGlobKey(key, '.auto') === name
  );
  return entry ? entry[1] : null;
}

type AutoCmd = {
  type?: string;
  data?: { commands?: AutoCmd[]; pathName?: string; name?: string };
};

/** Recursively collect {@code pathName} from PathPlanner {@code .auto} JSON. */
export function collectPathNamesFromAutoJson(raw: string): string[] {
  let root: { command?: AutoCmd };
  try {
    root = JSON.parse(raw) as { command?: AutoCmd };
  } catch {
    return [];
  }
  const out: string[] = [];
  function walk(cmd: AutoCmd | undefined) {
    if (!cmd || typeof cmd !== 'object') {
      return;
    }
    if (cmd.type === 'path' && cmd.data && typeof cmd.data.pathName === 'string') {
      out.push(cmd.data.pathName);
    }
    const inner = cmd.data?.commands;
    if (Array.isArray(inner)) {
      for (const c of inner) {
        walk(c);
      }
    }
  }
  walk(root.command);
  return out;
}
