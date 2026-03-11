// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you may modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.knn;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * KNN interpreter: loads a map of field points (x, y in meters), selects the
 * nearest point to the current pose, and publishes the selected index to
 * NetworkTables so the dashboard can align visualization.
 */
public class KnnInterpreter {
    private static final String KNN_MAP_FILENAME = "knn_map.json";
    private static final ObjectMapper OBJECT_MAPPER = new ObjectMapper();

    private final List<Pose2d> points = new ArrayList<>();
    private final NetworkTable table;
    private final IntegerPublisher selectedIndexPub;
    private int lastSelectedIndex = -1;

    public KnnInterpreter() {
        table = NetworkTableInstance.getDefault().getTable("KNN");
        selectedIndexPub = table.getIntegerTopic("selectedIndex").publish();
        loadMap();
    }

    /**
     * Loads the KNN map from deploy/knn_map.json. Expects a JSON array of
     * objects with "x", "y", and optional "headingDeg" / "rotation" fields.
     * If the file is missing or invalid, the map remains empty and
     * selectedIndex will stay -1.
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
                if (node.has("x") && node.has("y")) {
                    double x = node.get("x").asDouble();
                    double y = node.get("y").asDouble();
                    double headingDeg = 0.0;
                    if (node.has("headingDeg")) {
                        headingDeg = node.get("headingDeg").asDouble();
                    } else if (node.has("rotation")) {
                        headingDeg = node.get("rotation").asDouble();
                    }
                    points.add(new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg)));
                }
            }
        } catch (Exception e) {
            // Map remains empty; robot can still run without KNN
        }
    }

    private int findNearestIndex(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        int best = -1;
        double bestDistSq = Double.POSITIVE_INFINITY;
        for (int i = 0; i < points.size(); i++) {
            Pose2d p = points.get(i);
            double dx = x - p.getX();
            double dy = y - p.getY();
            double distSq = dx * dx + dy * dy;
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                best = i;
            }
        }
        return best;
    }

    private void publishSelection(int index) {
        lastSelectedIndex = index;
        selectedIndexPub.set(index);
    }

    /**
     * Updates the interpreter with the current robot pose, selects the nearest
     * map point (Euclidean distance in x, y), and publishes the selected index
     * to NetworkTables.
     *
     * @param pose current robot pose in field coordinates (meters)
     */
    public void update(Pose2d pose) {
        publishSelection(findNearestIndex(pose));
    }

    /**
     * Returns the nearest recorded pose to the supplied robot pose and
     * publishes the selected index for dashboard alignment.
     *
     * @param pose current robot pose in field coordinates (meters)
     * @return nearest recorded target pose, if one exists
     */
    public java.util.Optional<Pose2d> getNearestPose(Pose2d pose) {
        int best = findNearestIndex(pose);
        publishSelection(best);
        return best >= 0 ? java.util.Optional.of(points.get(best)) : java.util.Optional.empty();
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
