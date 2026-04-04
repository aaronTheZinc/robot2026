// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.knn;

import edu.wpi.first.math.geometry.Pose2d;

/** One KNN map sample: field pose (x, y, logged heading) plus shooter settings for that spot. */
public final class KnnMapPoint {
    private final Pose2d pose;
    private final double shooterRpm;
    private final double hoodDeg;

    public KnnMapPoint(Pose2d pose, double shooterRpm, double hoodDeg) {
        this.pose = pose;
        this.shooterRpm = shooterRpm;
        this.hoodDeg = hoodDeg;
    }

    public Pose2d pose() {
        return pose;
    }

    public double getShooterRpm() {
        return shooterRpm;
    }

    public double getHoodDeg() {
        return hoodDeg;
    }
}
