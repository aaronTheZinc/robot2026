// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you may modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.knn;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

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

    private final List<double[]> points = new ArrayList<>();
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
     * objects with "x" and "y" (meters). If the file is missing or invalid,
     * the map remains empty and selectedIndex will stay -1.
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
            Gson gson = new Gson();
            JsonArray arr = gson.fromJson(json, JsonArray.class);
            if (arr == null) {
                return;
            }
            for (JsonElement el : arr) {
                if (!el.isJsonObject()) {
                    continue;
                }
                JsonObject obj = el.getAsJsonObject();
                if (obj.has("x") && obj.has("y")) {
                    double x = obj.get("x").getAsDouble();
                    double y = obj.get("y").getAsDouble();
                    points.add(new double[] { x, y });
                }
            }
        } catch (Exception e) {
            // Map remains empty; robot can still run without KNN
        }
    }

    /**
     * Updates the interpreter with the current robot pose, selects the nearest
     * map point (Euclidean distance in x, y), and publishes the selected index
     * to NetworkTables.
     *
     * @param pose current robot pose in field coordinates (meters)
     */
    public void update(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        int best = -1;
        double bestDistSq = Double.POSITIVE_INFINITY;
        for (int i = 0; i < points.size(); i++) {
            double[] p = points.get(i);
            double dx = x - p[0];
            double dy = y - p[1];
            double distSq = dx * dx + dy * dy;
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                best = i;
            }
        }
        lastSelectedIndex = best;
        selectedIndexPub.set(best);
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
