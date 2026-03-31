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

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import frc.robot.DriveConstants;
import frc.robot.ShooterConstants;

/**
 * Loads {@code deploy/knn_map.json}, tracks the nearest map point to the robot pose, and
 * optionally publishes inverse-distance-weighted (IDW) shooter/hood estimates.
 *
 * <p>NetworkTables (table {@code KNN}):
 * <ul>
 *   <li>{@code selectedIndex} — nearest point index, or -1
 *   <li>{@code interpolateHoodEnabled} — mirrors {@link KnnConstants#kInterpolateHoodWhileDriving} (read-only; set in code)
 *   <li>{@code nearestShooterRpm}, {@code nearestHoodDeg} — values from the single nearest point
 *   <li>{@code interpolatedShooterRpm}, {@code interpolatedHoodDeg} — IDW blend from {@link KnnConstants#kIdwNearestCount}
 *       nearest points (same as nearest when only one point or at a sample pose)
 *   <li>{@code nearestHeadingDeg}, {@code interpolatedHeadingDeg} — saved vs circular IDW blend of logged {@code headingDeg}
 *   <li>{@code nearestAimHeadingDeg} — field heading to score toward the nearest sample's {@code shootTarget} (hub or field point)
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
    private final DoublePublisher nearestHeadingDegPub;
    private final DoublePublisher interpolatedHeadingDegPub;
    private final DoublePublisher nearestAimHeadingDegPub;

    private int lastSelectedIndex = -1;
    private double lastNearestRpm = ShooterConstants.kLeftBumperShotRpm;
    private double lastNearestHood = ShooterConstants.kLeftBumperShotHoodAngleDeg;
    private double lastInterpolatedRpm = ShooterConstants.kLeftBumperShotRpm;
    private double lastInterpolatedHood = ShooterConstants.kLeftBumperShotHoodAngleDeg;
    private double lastNearestHeadingDeg = 0.0;
    private double lastInterpolatedHeadingDeg = 0.0;
    /** Bearing to hold for KNN aim (hub or field target from nearest map row). */
    private Rotation2d lastNearestAimRotation = Rotation2d.kZero;

    public KnnInterpreter() {
        table = NetworkTableInstance.getDefault().getTable("KNN");
        selectedIndexPub = table.getIntegerTopic("selectedIndex").publish();
        interpolateHoodEnabledPub = table.getBooleanTopic("interpolateHoodEnabled").publish();
        nearestRpmPub = table.getDoubleTopic("nearestShooterRpm").publish();
        nearestHoodPub = table.getDoubleTopic("nearestHoodDeg").publish();
        interpolatedRpmPub = table.getDoubleTopic("interpolatedShooterRpm").publish();
        interpolatedHoodPub = table.getDoubleTopic("interpolatedHoodDeg").publish();
        nearestHeadingDegPub = table.getDoubleTopic("nearestHeadingDeg").publish();
        interpolatedHeadingDegPub = table.getDoubleTopic("interpolatedHeadingDeg").publish();
        nearestAimHeadingDegPub = table.getDoubleTopic("nearestAimHeadingDeg").publish();
        loadMap();
        interpolateHoodEnabledPub.set(KnnConstants.kInterpolateHoodWhileDriving);
    }

    /**
     * Loads the KNN map from deploy/knn_map.json. Expects a JSON array of objects with "x", "y",
     * optional "headingDeg" / "rotation", and optional "shooterRpm", "hoodDeg". Missing shooter
     * fields default to the center shot constants.
     */
    private void loadMap() {
        points.clear();
        Path deployDir = Filesystem.getDeployDirectory().toPath();
        Path mapPath = deployDir.resolve(KNN_MAP_FILENAME);
        if (!Files.isRegularFile(mapPath)) {
            return;
        }
        try {
            String json = Files.readString(mapPath);
            JsonNode root = OBJECT_MAPPER.readTree(json);
            if (root == null || !root.isArray()) {
                return;
            }
            for (JsonNode node : root) {
                if (!node.isObject()) {
                    continue;
                }
                if (!node.has("x") || !node.has("y")) {
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
                double hood = node.has("hoodDeg")
                        ? node.get("hoodDeg").asDouble()
                        : ShooterConstants.kLeftBumperShotHoodAngleDeg;
                KnnShootTarget shootTarget = parseShootTarget(node);
                points.add(new KnnMapPoint(
                        new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg)), rpm, hood, shootTarget));
            }
        } catch (Exception e) {
            // Map remains empty; robot can still run without KNN
        }
    }

    private static KnnShootTarget parseShootTarget(JsonNode mapRow) {
        if (!mapRow.has("shootTarget") || !mapRow.get("shootTarget").isObject()) {
            return new KnnShootTarget.Hub();
        }
        JsonNode st = mapRow.get("shootTarget");
        String kind = st.has("kind") ? st.get("kind").asText("hub") : "hub";
        if ("field".equals(kind) && st.has("x") && st.has("y")) {
            return new KnnShootTarget.Field(st.get("x").asDouble(), st.get("y").asDouble());
        }
        return new KnnShootTarget.Hub();
    }

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

    private void computeInterpolated(Pose2d pose) {
        if (points.isEmpty()) {
            lastInterpolatedRpm = ShooterConstants.kLeftBumperShotRpm;
            lastInterpolatedHood = ShooterConstants.kLeftBumperShotHoodAngleDeg;
            lastInterpolatedHeadingDeg = 0.0;
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
        int k = Math.min(KnnConstants.kIdwNearestCount, scored.size());
        double sumW = 0.0;
        double sumRpm = 0.0;
        double sumHood = 0.0;
        double sumSinHeading = 0.0;
        double sumCosHeading = 0.0;
        for (int j = 0; j < k; j++) {
            Scored s = scored.get(j);
            KnnMapPoint p = points.get(s.index);
            if (s.dist < 1e-9) {
                lastInterpolatedRpm = p.getShooterRpm();
                lastInterpolatedHood = p.getHoodDeg();
                lastInterpolatedHeadingDeg = p.pose().getRotation().getDegrees();
                return;
            }
            double w = 1.0 / (s.dist * s.dist + KnnConstants.kIdwEpsilonSq);
            sumW += w;
            sumRpm += w * p.getShooterRpm();
            sumHood += w * p.getHoodDeg();
            double rad = p.pose().getRotation().getRadians();
            sumSinHeading += w * Math.sin(rad);
            sumCosHeading += w * Math.cos(rad);
        }
        lastInterpolatedRpm = sumRpm / sumW;
        lastInterpolatedHood = sumHood / sumW;
        lastInterpolatedHeadingDeg = Math.toDegrees(Math.atan2(sumSinHeading, sumCosHeading));
    }

    private void publishSelection(int index) {
        lastSelectedIndex = index;
        selectedIndexPub.set(index);
        if (index >= 0 && index < points.size()) {
            KnnMapPoint p = points.get(index);
            lastNearestRpm = p.getShooterRpm();
            lastNearestHood = p.getHoodDeg();
            lastNearestHeadingDeg = p.pose().getRotation().getDegrees();
        } else {
            lastNearestRpm = ShooterConstants.kLeftBumperShotRpm;
            lastNearestHood = ShooterConstants.kLeftBumperShotHoodAngleDeg;
            lastNearestHeadingDeg = 0.0;
        }
        nearestRpmPub.set(lastNearestRpm);
        nearestHoodPub.set(lastNearestHood);
        nearestHeadingDegPub.set(lastNearestHeadingDeg);
    }

    private void updateNearestAimRotation(Pose2d robotPose, int nearestIndex) {
        if (points.isEmpty()) {
            lastNearestAimRotation = DriveConstants.rotationToFaceHub(robotPose);
        } else if (nearestIndex >= 0 && nearestIndex < points.size()) {
            KnnMapPoint p = points.get(nearestIndex);
            KnnShootTarget st = p.getShootTarget();
            if (st instanceof KnnShootTarget.Field f) {
                lastNearestAimRotation =
                        DriveConstants.rotationToFaceFieldPoint(robotPose, f.x(), f.y());
            } else {
                lastNearestAimRotation = DriveConstants.rotationToFaceHub(robotPose);
            }
        } else {
            lastNearestAimRotation = DriveConstants.rotationToFaceHub(robotPose);
        }
        nearestAimHeadingDegPub.set(lastNearestAimRotation.getDegrees());
    }

    /**
     * Updates nearest-index tracking, nearest-point shooter fields, and IDW interpolation for the
     * current pose. Call once per loop with the fused field pose.
     */
    public void update(Pose2d pose) {
        interpolateHoodEnabledPub.set(KnnConstants.kInterpolateHoodWhileDriving);
        int best = findNearestIndex(pose);
        publishSelection(best);
        updateNearestAimRotation(pose, best);
        computeInterpolated(pose);
        interpolatedRpmPub.set(lastInterpolatedRpm);
        interpolatedHoodPub.set(lastInterpolatedHood);
        interpolatedHeadingDegPub.set(lastInterpolatedHeadingDeg);
    }

    /** Nearest map point's saved field heading (deg), same as pose rotation in {@code knn_map.json}. */
    public Rotation2d getNearestHeadingRotation() {
        return Rotation2d.fromDegrees(lastNearestHeadingDeg);
    }

    /**
     * Heading to face the scoring target for the nearest KNN sample: hub, or {@code shootTarget} field
     * point (bearing from the <em>current</em> robot pose). Updated in {@link #update(Pose2d)}.
     */
    public Rotation2d getNearestAimRotation() {
        return lastNearestAimRotation;
    }

    /**
     * IDW circular blend of logged headings from the {@code kIdwNearestCount} nearest points (deg → rad).
     */
    public Rotation2d getInterpolatedHeadingRotation() {
        return Rotation2d.fromDegrees(lastInterpolatedHeadingDeg);
    }

    /**
     * Whether IDW hood tracking is active — {@link KnnConstants#kInterpolateHoodWhileDriving}.
     */
    public boolean isInterpolateEnabled() {
        return KnnConstants.kInterpolateHoodWhileDriving;
    }

    public double getInterpolatedRpm() {
        return lastInterpolatedRpm;
    }

    public double getInterpolatedHoodDeg() {
        return lastInterpolatedHood;
    }

    public double getNearestShooterRpm() {
        return lastNearestRpm;
    }

    public double getNearestHoodDeg() {
        return lastNearestHood;
    }

    /**
     * Returns the nearest recorded pose to the supplied robot pose and publishes the selected index
     * for dashboard alignment.
     */
    public Optional<Pose2d> getNearestPose(Pose2d pose) {
        int best = findNearestIndex(pose);
        publishSelection(best);
        updateNearestAimRotation(pose, best);
        computeInterpolated(pose);
        interpolatedRpmPub.set(lastInterpolatedRpm);
        interpolatedHoodPub.set(lastInterpolatedHood);
        interpolatedHeadingDegPub.set(lastInterpolatedHeadingDeg);
        return best >= 0 ? Optional.of(points.get(best).pose()) : Optional.empty();
    }

    /** Nearest map row including shooter fields (same index as {@code selectedIndex} when pose matches). */
    public Optional<KnnMapPoint> getNearestMapPoint(Pose2d pose) {
        int best = findNearestIndex(pose);
        publishSelection(best);
        updateNearestAimRotation(pose, best);
        computeInterpolated(pose);
        interpolatedRpmPub.set(lastInterpolatedRpm);
        interpolatedHoodPub.set(lastInterpolatedHood);
        interpolatedHeadingDegPub.set(lastInterpolatedHeadingDeg);
        return best >= 0 ? Optional.of(points.get(best)) : Optional.empty();
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
