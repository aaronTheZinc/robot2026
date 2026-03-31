import { HUB_FIELD_X_M, HUB_FIELD_Y_M } from './hubField';

/** Built-in alignment origin — always available, named Hub in UI. */
export const SIM_HUB_ORIGIN_ID = 'hub' as const;

export type SimNamedOrigin = {
  id: string;
  name: string;
  x: number;
  y: number;
};

export function newSimNamedOriginId(): string {
  return `origin-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
}

export function normalizeSimNamedOrigins(value: unknown): SimNamedOrigin[] {
  if (!Array.isArray(value)) {
    return [];
  }
  return value
    .map((item: unknown) => {
      if (!item || typeof item !== 'object') {
        return null;
      }
      const o = item as Record<string, unknown>;
      const id =
        typeof o.id === 'string' && o.id.length > 0 ? o.id : newSimNamedOriginId();
      const x = typeof o.x === 'number' && Number.isFinite(o.x) ? o.x : 0;
      const y = typeof o.y === 'number' && Number.isFinite(o.y) ? o.y : 0;
      const name =
        typeof o.name === 'string' && o.name.trim().length > 0 ? o.name.trim() : 'Origin';
      const out: SimNamedOrigin = { id, name, x, y };
      return out;
    })
    .filter((p): p is SimNamedOrigin => p !== null);
}

export function resolveSimAlignmentOrigin(
  originId: string,
  namedOrigins: readonly SimNamedOrigin[]
): { x: number; y: number; displayName: string } {
  if (originId === SIM_HUB_ORIGIN_ID) {
    return { x: HUB_FIELD_X_M, y: HUB_FIELD_Y_M, displayName: 'Hub' };
  }
  const o = namedOrigins.find((n) => n.id === originId);
  if (o) {
    return { x: o.x, y: o.y, displayName: o.name };
  }
  return { x: HUB_FIELD_X_M, y: HUB_FIELD_Y_M, displayName: 'Hub' };
}
