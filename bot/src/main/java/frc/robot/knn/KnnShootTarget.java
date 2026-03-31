// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.knn;

/**
 * Persisted scoring aim for a KNN map sample (matches dashboard {@code knn_map.json}{@code shootTarget}).
 */
public sealed interface KnnShootTarget permits KnnShootTarget.Hub, KnnShootTarget.Field {

    /** Default: face toward the hub from the current robot pose. */
    record Hub() implements KnnShootTarget {}

    /** Face toward a field point (meters, same frame as odometry / PathPlanner blue). */
    record Field(double x, double y) implements KnnShootTarget {}
}
