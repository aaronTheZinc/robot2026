// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.knn;

/** Tunables for KNN map loading and inverse-distance interpolation (IDW). */
public final class KnnConstants {
    private KnnConstants() {}

    /**
     * When true, hood follows smoothed IDW from the map while driving (see {@code RobotContainer#applyKnnHoodInterpolation}).
     */
    public static final boolean kInterpolateHoodWhileDriving = true;

    /** Max magnitude of hood setpoint change per second when tracking IDW (deg/s). */
    public static final double kHoodInterpMaxRateDegPerSec = 22.0;

    /** Max shooter RPM change per second when tracking map targets (reduces oscillation). */
    public static final double kShooterRpmInterpMaxRatePerSec = 350.0;

    /** How many nearest map points participate in IDW (must be >= 1). */
    public static final int kIdwNearestCount = 4;

    /** Beyond this distance to the geometric nearest map point, IDW uses only this many neighbors. */
    public static final double kIdwFarFewNeighborsBeyondM = 2.2;

    /** Neighbor count when far (smaller = stabler when extrapolating between distant clusters). */
    public static final int kIdwFarNeighborCount = 2;

    /**
     * Blend factor toward pure IDW is 0 at this distance and 1 at the robot (linear in between). Past this,
     * output follows the nearest map row more than IDW to kill far-field jitter.
     */
    public static final double kIdwBlendFullNeighborBeyondM = 3.5;

    /** Added to squared distance in IDW weights to avoid blow-up at d ≈ 0 (m²). */
    public static final double kIdwEpsilonSq = 0.02;

    /** Map JSON rows at or below this RPM are ignored (e.g. placeholder 0,0 calibration points). */
    public static final double kMapMinCalibrationRpm = 1.0;

    /** Merge map entries whose X/Y are within this (m) into one point (average rpm/hood). */
    public static final double kMapMergePositionEpsilonM = 0.05;

    /**
     * Minimum distance advantage (m) a map point must have over the currently selected point before the
     * selected index switches. Reduces hood/RPM flicker when pose noise or symmetric layouts toggle the
     * geometric nearest neighbor.
     */
    public static final double kNearestIndexHysteresisM = 0.12;

    /** Field length along X (m), WPILib blue origin — match dashboard {@code FIELD_LENGTH_M}. */
    public static final double kFieldLengthXMeters = 16.46;

    /**
     * Field width along Y (m) for KNN mirror — must match dashboard {@code FIELD_WIDTH_M} when mirroring
     * points in {@code knnFieldMirror.ts} (reflection {@code y' = W - y}).
     */
    public static final double kFieldWidthYMeters = 8.23;

    /**
     * When true and DS alliance is Red, KNN lookup uses pose {@code (L - x, W - y)} so a map tuned from the
     * blue half matches symmetric red-side positions (same idea as hub X/Y in {@link frc.robot.DriveConstants}).
     * Set false if {@code knn_map.json} uses absolute WPIBlue coordinates covering both halves of the field.
     */
    public static final boolean kMirrorPoseAcrossFieldForRedAllianceKnnLookup = true;
}
