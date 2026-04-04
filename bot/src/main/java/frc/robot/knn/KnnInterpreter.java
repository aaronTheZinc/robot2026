// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.knn;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import frc.robot.ShooterConstants;

/**
 * Loads {@code deploy/knn_map.json}, finds the nearest map point to the robot pose, and publishes
 * inverse-distance-weighted (IDW) shooter RPM and hood. Map rows with non-positive RPM are dropped; nearby
 * duplicate positions are merged (average). Outputs are clamped to the map's hood/RPM envelope, blended toward
 * the nearest row when far from data ({@link KnnConstants#kIdwBlendFullNeighborBeyondM}), then slewed with
 * {@link KnnConstants#kHoodInterpMaxRateDegPerSec} / {@link KnnConstants#kShooterRpmInterpMaxRatePerSec}.
 * Extra JSON fields (e.g. {@code shootTarget}) are ignored on-robot.
 *
 * <p>NetworkTables (table {@code KNN}):
 * <ul>
 *   <li>{@code selectedIndex} — nearest point index, or -1
 *   <li>{@code interpolateHoodEnabled} — mirrors {@link KnnConstants#kInterpolateHoodWhileDriving} (read-only)
 *   <li>{@code nearestShooterRpm}, {@code nearestHoodDeg} — from the single nearest point
 *   <li>{@code interpolatedShooterRpm}, {@code interpolatedHoodDeg} — raw IDW blend from {@link KnnConstants#kIdwNearestCount}
 *       nearest points (pre-slew target); {@code smoothedHoodDeg} / {@code smoothedShooterRpm} are what the robot uses
 * </ul>
 */
public class KnnInterpreter {
    private static final String KNN_MAP_FILENAME = "knn_map.json";
    private static final ObjectMapper OBJECT_MAPPER = new ObjectMapper();

    private final List<KnnMapPoint> points = new ArrayList<>();
    private final NetworkTable table;
    private final IntegerPublisher selectedIndexPub;
    private final BooleanPublisher interpolateHoodEnabledPub;
    private final DoublePublisher nearestRpmPub;
    private final DoublePublisher nearestHoodPub;
    private final DoublePublisher interpolatedRpmPub;
    private final DoublePublisher interpolatedHoodPub;
    private final DoublePublisher smoothedHoodPub;
    private final DoublePublisher smoothedRpmPub;

    private int lastSelectedIndex = -1;
    private double lastNearestRpm = ShooterConstants.kLeftBumperShotRpm;
    private double lastNearestHood = ShooterConstants.kLeftBumperShotHoodAngleDeg;
    /** Blended + map-clamped targets before temporal slew (hood / RPM). */
    private double lastInterpolatedRpm = ShooterConstants.kLeftBumperShotRpm;
    private double lastInterpolatedHood = ShooterConstants.kLeftBumperShotHoodAngleDeg;
    /** Rate-limited hood toward {@link #lastInterpolatedHood}; use for physical setpoint. */
    private double smoothedHoodDeg = ShooterConstants.kLeftBumperShotHoodAngleDeg;
    private double hoodSmoothLastTimeSec = -1.0;
    /** Rate-limited RPM toward {@link #lastInterpolatedRpm}. */
    private double smoothedRpm = ShooterConstants.kLeftBumperShotRpm;
    private double rpmSmoothLastTimeSec = -1.0;

    /** From loaded map (after merge/filter); IDW outputs are clamped to these ranges. */
    private double mapRpmMin = ShooterConstants.kLeftBumperShotRpm;
    private double mapRpmMax = ShooterConstants.kLeftBumperShotRpm;
    private double mapHoodMin = ShooterConstants.kLeftBumperShotHoodAngleDeg;
    private double mapHoodMax = ShooterConstants.kLeftBumperShotHoodAngleDeg;

    public KnnInterpreter() {
        table = NetworkTableInstance.getDefault().getTable("KNN");
        selectedIndexPub = table.getIntegerTopic("selectedIndex").publish();
        interpolateHoodEnabledPub = table.getBooleanTopic("interpolateHoodEnabled").publish();
        nearestRpmPub = table.getDoubleTopic("nearestShooterRpm").publish();
        nearestHoodPub = table.getDoubleTopic("nearestHoodDeg").publish();
        interpolatedRpmPub = table.getDoubleTopic("interpolatedShooterRpm").publish();
        interpolatedHoodPub = table.getDoubleTopic("interpolatedHoodDeg").publish();
        smoothedHoodPub = table.getDoubleTopic("smoothedHoodDeg").publish();
        smoothedRpmPub = table.getDoubleTopic("smoothedShooterRpm").publish();
        loadMap();
        interpolateHoodEnabledPub.set(KnnConstants.kInterpolateHoodWhileDriving);
    }

    /**
     * Loads the KNN map from deploy/knn_map.json. Drops non-calibrated rows ({@code shooterRpm} at or below
     * {@link KnnConstants#kMapMinCalibrationRpm}), merges nearby duplicate X/Y into one point (average rpm/hood).
     */
    private void loadMap() {
        points.clear();
        Path deployDir = Filesystem.getDeployDirectory().toPath();
        Path mapPath = deployDir.resolve(KNN_MAP_FILENAME);
        if (!Files.isRegularFile(mapPath)) {
            recomputeMapBoundsFromPoints();
            return;
        }
        try {
            String json = Files.readString(mapPath);
            JsonNode root = OBJECT_MAPPER.readTree(json);
            if (root != null && root.isArray()) {
                Map<String, MergeCell> cells = new HashMap<>();
                for (JsonNode node : root) {
                    if (!node.isObject() || !node.has("x") || !node.has("y")) {
                        continue;
                    }
                    double x = node.get("x").asDouble();
                    double y = node.get("y").asDouble();
                    double headingDeg = 0.0;
                    if (node.has("headingDeg")) {
                        headingDeg = node.get("headingDeg").asDouble();
                    } else if (node.has("rotation")) {
                        headingDeg = node.get("rotation").asDouble();
                    }
                    double rpm = node.has("shooterRpm")
                            ? node.get("shooterRpm").asDouble()
                            : ShooterConstants.kLeftBumperShotRpm;
                    if (rpm <= KnnConstants.kMapMinCalibrationRpm) {
                        continue;
                    }
                    double hood = node.has("hoodDeg")
                            ? node.get("hoodDeg").asDouble()
                            : ShooterConstants.kLeftBumperShotHoodAngleDeg;
                    String key = mergePositionKey(x, y);
                    cells.computeIfAbsent(key, k -> new MergeCell()).add(x, y, rpm, hood, headingDeg);
                }
                for (MergeCell c : cells.values()) {
                    points.add(c.toMapPoint());
                }
            }
        } catch (Exception e) {
            points.clear();
        }
        recomputeMapBoundsFromPoints();
    }

    private static String mergePositionKey(double x, double y) {
        double g = KnnConstants.kMapMergePositionEpsilonM;
        long qx = Math.round(x / g);
        long qy = Math.round(y / g);
        return qx + ":" + qy;
    }

    private static final class MergeCell {
        private double sumX;
        private double sumY;
        private double sumRpm;
        private double sumHood;
        private int n;
        private double headingDeg;

        private void add(double x, double y, double rpm, double hood, double hDeg) {
            if (n == 0) {
                headingDeg = hDeg;
            }
            sumX += x;
            sumY += y;
            sumRpm += rpm;
            sumHood += hood;
            n++;
        }

        private KnnMapPoint toMapPoint() {
            double inv = 1.0 / n;
            return new KnnMapPoint(
                    new Pose2d(sumX * inv, sumY * inv, Rotation2d.fromDegrees(headingDeg)),
                    sumRpm * inv,
                    sumHood * inv);
        }
    }

    private void recomputeMapBoundsFromPoints() {
        if (points.isEmpty()) {
            mapRpmMin = mapRpmMax = ShooterConstants.kLeftBumperShotRpm;
            mapHoodMin = mapHoodMax = ShooterConstants.kLeftBumperShotHoodAngleDeg;
            return;
        }
        mapRpmMin = Double.POSITIVE_INFINITY;
        mapRpmMax = Double.NEGATIVE_INFINITY;
        mapHoodMin = Double.POSITIVE_INFINITY;
        mapHoodMax = Double.NEGATIVE_INFINITY;
        for (KnnMapPoint p : points) {
            double r = p.getShooterRpm();
            double h = p.getHoodDeg();
            mapRpmMin = Math.min(mapRpmMin, r);
            mapRpmMax = Math.max(mapRpmMax, r);
            mapHoodMin = Math.min(mapHoodMin, h);
            mapHoodMax = Math.max(mapHoodMax, h);
        }
    }

    private double clampToMapRpm(double rpm) {
        return Math.max(mapRpmMin, Math.min(mapRpmMax, rpm));
    }

    private double clampToMapHood(double hood) {
        return Math.max(mapHoodMin, Math.min(mapHoodMax, hood));
    }

    private double clampShooterRpmSetpoint(double rpm) {
        double v = clampToMapRpm(rpm);
        return Math.max(ShooterConstants.kShooterMinRpm, Math.min(ShooterConstants.kShooterMaxRpm, v));
    }

    private double clampHoodMechanism(double hood) {
        double v = clampToMapHood(hood);
        return Math.max(ShooterConstants.kHoodMinAngleDeg, Math.min(ShooterConstants.kHoodMaxAngleDeg, v));
    }

    /** Geometric nearest index (no hysteresis). */
    private int findNearestIndex(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        int best = -1;
        double bestDistSq = Double.POSITIVE_INFINITY;
        for (int i = 0; i < points.size(); i++) {
            KnnMapPoint p = points.get(i);
            double dx = x - p.pose().getX();
            double dy = y - p.pose().getY();
            double distSq = dx * dx + dy * dy;
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                best = i;
            }
        }
        return best;
    }

    private double distanceToPoint(Pose2d pose, int index) {
        if (index < 0 || index >= points.size()) {
            return Double.POSITIVE_INFINITY;
        }
        KnnMapPoint p = points.get(index);
        double dx = pose.getX() - p.pose().getX();
        double dy = pose.getY() - p.pose().getY();
        return Math.hypot(dx, dy);
    }

    /**
     * Like {@link #findNearestIndex} but keeps the previous selection until another point is clearly closer
     * by {@link KnnConstants#kNearestIndexHysteresisM}.
     */
    private int findStableNearestIndex(Pose2d pose) {
        int rawBest = findNearestIndex(pose);
        if (rawBest < 0) {
            return -1;
        }
        if (lastSelectedIndex < 0 || lastSelectedIndex >= points.size()) {
            return rawBest;
        }
        if (rawBest == lastSelectedIndex) {
            return rawBest;
        }
        double dRaw = distanceToPoint(pose, rawBest);
        double dLast = distanceToPoint(pose, lastSelectedIndex);
        double margin = KnnConstants.kNearestIndexHysteresisM;
        if (dRaw < dLast - margin) {
            return rawBest;
        }
        return lastSelectedIndex;
    }

    private void computeInterpolated(Pose2d pose) {
        if (points.isEmpty()) {
            lastInterpolatedRpm =
                    clampShooterRpmSetpoint(ShooterConstants.kLeftBumperShotRpm);
            lastInterpolatedHood = clampHoodMechanism(ShooterConstants.kLeftBumperShotHoodAngleDeg);
            return;
        }
        record Scored(int index, double dist) {}
        List<Scored> scored = new ArrayList<>(points.size());
        for (int i = 0; i < points.size(); i++) {
            KnnMapPoint p = points.get(i);
            double dx = pose.getX() - p.pose().getX();
            double dy = pose.getY() - p.pose().getY();
            scored.add(new Scored(i, Math.hypot(dx, dy)));
        }
        scored.sort(Comparator.comparingDouble(s -> s.dist));
        Scored closest = scored.get(0);
        double nearestDist = closest.dist;
        KnnMapPoint nearestPt = points.get(closest.index);
        double nearestRpm = nearestPt.getShooterRpm();
        double nearestHood = nearestPt.getHoodDeg();

        if (nearestDist < 1e-9) {
            lastInterpolatedRpm = clampShooterRpmSetpoint(nearestRpm);
            lastInterpolatedHood = clampHoodMechanism(nearestHood);
            return;
        }

        int kWant = Math.min(KnnConstants.kIdwNearestCount, scored.size());
        if (nearestDist > KnnConstants.kIdwFarFewNeighborsBeyondM) {
            kWant = Math.min(KnnConstants.kIdwFarNeighborCount, kWant);
        }
        int kEff = Math.max(1, kWant);

        double sumW = 0.0;
        double sumRpm = 0.0;
        double sumHood = 0.0;
        for (int j = 0; j < kEff; j++) {
            Scored s = scored.get(j);
            KnnMapPoint p = points.get(s.index);
            if (s.dist < 1e-9) {
                lastInterpolatedRpm = clampShooterRpmSetpoint(p.getShooterRpm());
                lastInterpolatedHood = clampHoodMechanism(p.getHoodDeg());
                return;
            }
            double w = 1.0 / (s.dist * s.dist + KnnConstants.kIdwEpsilonSq);
            sumW += w;
            sumRpm += w * p.getShooterRpm();
            sumHood += w * p.getHoodDeg();
        }
        double idwRpm = clampToMapRpm(sumRpm / sumW);
        double idwHood = clampToMapHood(sumHood / sumW);

        double far = KnnConstants.kIdwBlendFullNeighborBeyondM;
        double idwFrac = far > 1e-6 ? Math.max(0.0, Math.min(1.0, 1.0 - nearestDist / far)) : 1.0;
        double outRpm = idwFrac * idwRpm + (1.0 - idwFrac) * nearestRpm;
        double outHood = idwFrac * idwHood + (1.0 - idwFrac) * nearestHood;
        lastInterpolatedRpm = clampShooterRpmSetpoint(outRpm);
        lastInterpolatedHood = clampHoodMechanism(outHood);
    }

    private void updateSmoothedHood() {
        double target = lastInterpolatedHood;
        double now = Timer.getFPGATimestamp();
        if (hoodSmoothLastTimeSec < 0.0) {
            hoodSmoothLastTimeSec = now;
            smoothedHoodDeg = target;
            return;
        }
        double dt = Math.max(1e-3, Math.min(now - hoodSmoothLastTimeSec, 0.1));
        hoodSmoothLastTimeSec = now;
        double maxStep = KnnConstants.kHoodInterpMaxRateDegPerSec * dt;
        double err = target - smoothedHoodDeg;
        if (err > maxStep) {
            err = maxStep;
        } else if (err < -maxStep) {
            err = -maxStep;
        }
        smoothedHoodDeg += err;
        smoothedHoodDeg = clampHoodMechanism(smoothedHoodDeg);
    }

    private void updateSmoothedRpm() {
        double target = lastInterpolatedRpm;
        double now = Timer.getFPGATimestamp();
        if (rpmSmoothLastTimeSec < 0.0) {
            rpmSmoothLastTimeSec = now;
            smoothedRpm = target;
            return;
        }
        double dt = Math.max(1e-3, Math.min(now - rpmSmoothLastTimeSec, 0.1));
        rpmSmoothLastTimeSec = now;
        double maxStep = KnnConstants.kShooterRpmInterpMaxRatePerSec * dt;
        double err = target - smoothedRpm;
        if (err > maxStep) {
            err = maxStep;
        } else if (err < -maxStep) {
            err = -maxStep;
        }
        smoothedRpm += err;
        smoothedRpm =
                Math.max(
                        ShooterConstants.kShooterMinRpm,
                        Math.min(ShooterConstants.kShooterMaxRpm, smoothedRpm));
    }

    /**
     * Jumps the smoothed hood to the current map target (after {@link #update} / {@link #getNearestPose} refresh).
     * Call when starting a shot so the hood does not lag behind pose.
     */
    public void snapSmoothedHoodToInterpolated() {
        smoothedHoodDeg = clampHoodMechanism(lastInterpolatedHood);
        hoodSmoothLastTimeSec = Timer.getFPGATimestamp();
    }

    /** Jumps smoothed RPM to the current map target (call with {@link #snapSmoothedHoodToInterpolated} on shot start). */
    public void snapSmoothedRpmToInterpolated() {
        smoothedRpm = clampShooterRpmSetpoint(lastInterpolatedRpm);
        rpmSmoothLastTimeSec = Timer.getFPGATimestamp();
    }

    private void publishSelection(int index) {
        lastSelectedIndex = index;
        selectedIndexPub.set(index);
        if (index >= 0 && index < points.size()) {
            KnnMapPoint p = points.get(index);
            lastNearestRpm = p.getShooterRpm();
            lastNearestHood = p.getHoodDeg();
        } else {
            lastNearestRpm = ShooterConstants.kLeftBumperShotRpm;
            lastNearestHood = ShooterConstants.kLeftBumperShotHoodAngleDeg;
        }
        nearestRpmPub.set(lastNearestRpm);
        nearestHoodPub.set(lastNearestHood);
    }

    private void refreshForPose(Pose2d pose) {
        interpolateHoodEnabledPub.set(KnnConstants.kInterpolateHoodWhileDriving);
        int best = findStableNearestIndex(pose);
        publishSelection(best);
        computeInterpolated(pose);
        updateSmoothedHood();
        updateSmoothedRpm();
        interpolatedRpmPub.set(lastInterpolatedRpm);
        interpolatedHoodPub.set(lastInterpolatedHood);
        smoothedHoodPub.set(smoothedHoodDeg);
        smoothedRpmPub.set(smoothedRpm);
    }

    /**
     * Updates nearest-index tracking and IDW hood / RPM for the current pose. Call once per loop with the
     * fused field pose.
     */
    public void update(Pose2d pose) {
        refreshForPose(pose);
    }

    /**
     * Whether IDW hood tracking is active — {@link KnnConstants#kInterpolateHoodWhileDriving}.
     */
    public boolean isInterpolateEnabled() {
        return KnnConstants.kInterpolateHoodWhileDriving;
    }

    /** Map-based RPM target with slew limiting (what the shooter should track). */
    public double getInterpolatedRpm() {
        return smoothedRpm;
    }

    /** Blended map RPM before slew (NetworkTables {@code interpolatedShooterRpm} matches this each update). */
    public double getBlendedRpmTarget() {
        return lastInterpolatedRpm;
    }

    public double getInterpolatedHoodDeg() {
        return lastInterpolatedHood;
    }

    /** Hood setpoint for shooting / {@code applyKnnHoodInterpolation}: IDW with rate-limited smoothing. */
    public double getSmoothedInterpolatedHoodDeg() {
        return smoothedHoodDeg;
    }

    public double getNearestShooterRpm() {
        return lastNearestRpm;
    }

    public double getNearestHoodDeg() {
        return lastNearestHood;
    }

    /**
     * Returns the nearest recorded pose to the supplied robot pose and publishes the selected index for
     * dashboard alignment.
     */
    public Optional<Pose2d> getNearestPose(Pose2d pose) {
        refreshForPose(pose);
        return lastSelectedIndex >= 0
                ? Optional.of(points.get(lastSelectedIndex).pose())
                : Optional.empty();
    }

    /** Nearest map row (same index as {@code selectedIndex} after this call). */
    public Optional<KnnMapPoint> getNearestMapPoint(Pose2d pose) {
        refreshForPose(pose);
        return lastSelectedIndex >= 0
                ? Optional.of(points.get(lastSelectedIndex))
                : Optional.empty();
    }

    /** Returns the number of loaded map points. */
    public int getMapSize() {
        return points.size();
    }

    /** Returns the last selected point index, or -1 if none. */
    public int getLastSelectedIndex() {
        return lastSelectedIndex;
    }
}
