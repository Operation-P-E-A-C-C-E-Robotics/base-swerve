// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.field.FieldConstants;

/**
 * Utility funcitons for flipping from the blue to red alliance. By default, all translations and
 * poses in {@link FieldConstants} are stored with the origin at the rightmost point on the blue
 * alliance wall.
 */
public class AllianceFlipUtil {
    /** Flips a translation to the correct side of the field based on the current alliance color. */
    public static Translation2d apply(Translation2d translation) {
        if (shouldFlip()) {
            return new Translation2d(FieldConstants.fieldLength - translation.getX(), translation.getY());
        } else {
            return translation;
        }
    }

    public static Translation3d apply(Translation3d translation) {
        if (shouldFlip()) {
            return new Translation3d(FieldConstants.fieldLength - translation.getX(), translation.getY(), translation.getZ());
        } else {
            return translation;
        }
    }

    /** Flips a rotation based on the current alliance color. */
    public static Rotation2d apply(Rotation2d rotation) {
        if (shouldFlip()) {
            return new Rotation2d(-rotation.getCos(), rotation.getSin());
        } else {
            return rotation;
        }
    }

    /** Flips a pose to the correct side of the field based on the current alliance color. */
    public static Pose2d apply(Pose2d pose) {
        if (shouldFlip()) {
            return new Pose2d(
                    FieldConstants.fieldLength - pose.getX(),
                    pose.getY(),
                    new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
        } else {
            return pose;
        }
    }

    /**
     * Flips a trajectory state to the correct side of the field based on the current alliance color.
     */
    public static Trajectory.State apply(Trajectory.State state) {
        if (shouldFlip()) {
            return new Trajectory.State(
                    state.timeSeconds,
                    state.velocityMetersPerSecond,
                    state.accelerationMetersPerSecondSq,
                    new Pose2d(
                            FieldConstants.fieldLength - state.poseMeters.getX(),
                            state.poseMeters.getY(),
                            new Rotation2d(
                                    -state.poseMeters.getRotation().getCos(),
                                    state.poseMeters.getRotation().getSin())),
                    -state.curvatureRadPerMeter);
        } else {
            return state;
        }
    }

    private static boolean shouldFlip() {
        return DriverStation.getAlliance().get() == Alliance.Red;
    }
}