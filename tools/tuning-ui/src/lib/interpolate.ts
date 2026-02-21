export type InterpolationInput = {
  x: number;
  y: number;
  rotation: number;
};

export type InterpolationOutput = {
  turret: number;
  shooter: number;
  hood: number;
};

export type SamplePoint = InterpolationInput &
  InterpolationOutput & {
    timestampMs: number;
    id: string;
  };

export type KNNConfig = {
  k: number;
  weightX: number;
  weightY: number;
  weightRotation: number;
  power: number;
};

const DEFAULT_OUTPUT: InterpolationOutput = {
  turret: 0,
  shooter: 0,
  hood: 0,
};

const clamp = (value: number, min: number, max: number) =>
  Math.min(max, Math.max(min, value));

const angleDeltaDeg = (a: number, b: number) => {
  const diff = ((a - b + 540) % 360) - 180;
  return Math.abs(diff);
};

const distance = (a: InterpolationInput, b: InterpolationInput, cfg: KNNConfig) => {
  const dx = (a.x - b.x) * cfg.weightX;
  const dy = (a.y - b.y) * cfg.weightY;
  const dRot = angleDeltaDeg(a.rotation, b.rotation) * cfg.weightRotation;
  return Math.sqrt(dx * dx + dy * dy + dRot * dRot);
};

export function interpolateKNN(
  input: InterpolationInput,
  samples: SamplePoint[],
  config: KNNConfig
): InterpolationOutput | null {
  if (samples.length === 0) {
    return null;
  }

  const k = clamp(Math.round(config.k), 1, samples.length);
  const scored = samples
    .map((sample) => ({ sample, d: distance(input, sample, config) }))
    .sort((a, b) => a.d - b.d)
    .slice(0, k);

  if (scored[0]?.d === 0) {
    return {
      turret: scored[0].sample.turret,
      shooter: scored[0].sample.shooter,
      hood: scored[0].sample.hood,
    };
  }

  const power = Math.max(0.0001, config.power);
  let totalWeight = 0;
  let turret = 0;
  let shooter = 0;
  let hood = 0;

  for (const { sample, d } of scored) {
    const weight = 1 / Math.pow(Math.max(d, 0.000001), power);
    totalWeight += weight;
    turret += sample.turret * weight;
    shooter += sample.shooter * weight;
    hood += sample.hood * weight;
  }

  if (totalWeight === 0) {
    return { ...DEFAULT_OUTPUT };
  }

  return {
    turret: turret / totalWeight,
    shooter: shooter / totalWeight,
    hood: hood / totalWeight,
  };
}
