// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.knn;

/** Tunables for KNN map loading and inverse-distance interpolation (IDW). */
public final class KnnConstants {
    private KnnConstants() {}

    /**
     * When true, hood angle follows IDW from the map while driving (see {@code RobotContainer#applyKnnHoodInterpolation}).
     * Toggle here in code — not controlled from NetworkTables.
     */
    public static final boolean kInterpolateHoodWhileDriving = true;

    /** How many nearest map points participate in IDW (must be >= 1). */
    public static final int kIdwNearestCount = 4;

    /** Added to squared distance in IDW weights to avoid blow-up at d ≈ 0 (m²). */
    public static final double kIdwEpsilonSq = 1e-6;
}
