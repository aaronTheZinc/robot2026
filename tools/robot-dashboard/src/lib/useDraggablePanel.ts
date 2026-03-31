import {
  useCallback,
  useRef,
  useState,
  type CSSProperties,
  type PointerEvent,
  type RefObject,
} from 'react';

type Pos = { left: number; top: number };

type DragStart = { clientX: number; clientY: number; left: number; top: number };

/**
 * Drag a panel by a handle; positions are relative to boundsRef (e.g. simulation section).
 * Until the first drag, CSS layout is unchanged.
 */
export function useDraggablePanel(boundsRef: RefObject<HTMLElement | null>) {
  const [pos, setPos] = useState<Pos | null>(null);
  const dragStart = useRef<DragStart | null>(null);

  const onPointerMove = useCallback(
    (e: PointerEvent) => {
      if (!dragStart.current || !boundsRef.current) return;
      const panel = (e.currentTarget as HTMLElement).closest('[data-draggable-panel]') as HTMLElement | null;
      if (!panel) return;
      const b = boundsRef.current.getBoundingClientRect();
      const dx = e.clientX - dragStart.current.clientX;
      const dy = e.clientY - dragStart.current.clientY;
      const pw = panel.offsetWidth;
      const ph = panel.offsetHeight;
      let left = dragStart.current.left + dx;
      let top = dragStart.current.top + dy;
      left = Math.max(0, Math.min(left, b.width - pw));
      top = Math.max(0, Math.min(top, b.height - ph));
      setPos({ left, top });
    },
    [boundsRef]
  );

  const endDrag = useCallback((e: PointerEvent) => {
    dragStart.current = null;
    try {
      (e.currentTarget as HTMLElement).releasePointerCapture(e.pointerId);
    } catch {
      /* already released */
    }
  }, []);

  const onPointerDown = useCallback(
    (e: PointerEvent) => {
      if (e.button !== 0) return;
      const handle = e.currentTarget as HTMLElement;
      const panel = handle.closest('[data-draggable-panel]') as HTMLElement | null;
      const bounds = boundsRef.current;
      if (!panel || !bounds) return;
      e.preventDefault();
      e.stopPropagation();

      const b = bounds.getBoundingClientRect();
      const pr = panel.getBoundingClientRect();
      const startLeft = pos?.left ?? pr.left - b.left;
      const startTop = pos?.top ?? pr.top - b.top;
      if (pos === null) {
        setPos({ left: startLeft, top: startTop });
      }
      dragStart.current = {
        clientX: e.clientX,
        clientY: e.clientY,
        left: startLeft,
        top: startTop,
      };
      handle.setPointerCapture(e.pointerId);
    },
    [boundsRef, pos]
  );

  const panelStyle: CSSProperties | undefined = pos
    ? {
        left: pos.left,
        top: pos.top,
        right: 'auto',
        bottom: 'auto',
        transform: 'none',
      }
    : undefined;

  const handleProps = {
    onPointerDown,
    onPointerMove,
    onPointerUp: endDrag,
    onPointerCancel: endDrag,
  } as const;

  return {
    panelProps: {
      'data-draggable-panel': true,
      style: panelStyle,
    } as const,
    handleProps,
  };
}
