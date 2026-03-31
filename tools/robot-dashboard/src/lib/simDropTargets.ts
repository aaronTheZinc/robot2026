import { SIM_HUB_ORIGIN_ID } from './simFieldOrigins';

export type SimDropTarget = {
  id: string;
  x: number;
  y: number;
  /** Display name for this aim point. */
  label: string;
  /** Named alignment origin id: {@link SIM_HUB_ORIGIN_ID} or a custom origin id. Default Hub. */
  alignmentOriginId: string;
};

export function newSimDropTargetId(): string {
  return `drop-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
}

export function normalizeSimDropTargets(value: unknown): SimDropTarget[] {
  if (!Array.isArray(value)) {
    return [];
  }
  return value
    .map((item: unknown) => {
      if (!item || typeof item !== 'object') {
        return null;
      }
      const o = item as Record<string, unknown>;
      const id = typeof o.id === 'string' && o.id.length > 0 ? o.id : newSimDropTargetId();
      const x = typeof o.x === 'number' && Number.isFinite(o.x) ? o.x : 0;
      const y = typeof o.y === 'number' && Number.isFinite(o.y) ? o.y : 0;
      const label =
        typeof o.label === 'string' && o.label.trim().length > 0 ? o.label.trim() : 'Drop';
      const alignmentOriginId =
        typeof o.alignmentOriginId === 'string' && o.alignmentOriginId.length > 0
          ? o.alignmentOriginId
          : SIM_HUB_ORIGIN_ID;
      const out: SimDropTarget = { id, x, y, label, alignmentOriginId };
      return out;
    })
    .filter((p): p is SimDropTarget => p !== null);
}
