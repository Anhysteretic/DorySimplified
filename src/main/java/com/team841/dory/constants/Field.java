package com.team841.dory.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Field Constants. Stores the Reef positions and also the scoring poses. All poses follow wpilib coordinate system
 */
public class Field {

    public static class Positions {
        public static class Reef {
            public static final Pose2d bluePose2d =
                    new Pose2d(Meters.of(4.489337), Meters.of(4.025900), Rotation2d.kZero);
            public static final Translation2d blueTranslation2d =
                    new Translation2d(Meters.of(4.489337), Meters.of(4.025900));
            public static final Pose2d redPose2d =
                    new Pose2d(Meters.of(13.059), Meters.of(4.025900), Rotation2d.kZero);
            public static final Translation2d redTranslation2d =
                    new Translation2d(Meters.of(13.059), Meters.of(4.025900));
        }
    }

    // Scoring poses at each peg, following the game manual letter naming of each location
    // Each pose is for the blue reef and can be mirrored for the red reef by calling `getPoseRed()`
    public enum ScoringPositions {
        A(new Pose2d(3.208, 4.178, new Rotation2d(0))),
        B(new Pose2d(3.218, 3.845, new Rotation2d(0))),
        C(new Pose2d(3.720, 3.031, new Rotation2d(Units.degreesToRadians(60)))),
        D(new Pose2d(4.000, 2.865, new Rotation2d(Units.degreesToRadians(60)))),
        E(new Pose2d(4.988, 2.838, new Rotation2d(Units.degreesToRadians(120)))),
        F(new Pose2d(5.270, 3.024, new Rotation2d(Units.degreesToRadians(120)))),
        G(new Pose2d(5.773, 3.851, new Rotation2d(Units.degreesToRadians(180)))),
        H(new Pose2d(5.751, 4.197, new Rotation2d(Units.degreesToRadians(180)))),
        I(new Pose2d(5.259, 5.019, new Rotation2d(Units.degreesToRadians(240)))),
        J(new Pose2d(4.970, 5.204, new Rotation2d(Units.degreesToRadians(240)))),
        K(new Pose2d(4.000, 5.227, new Rotation2d(Units.degreesToRadians(300)))),
        L(new Pose2d(3.720, 5.046, new Rotation2d(Units.degreesToRadians(300))));

        private final Pose2d pose;

        ScoringPositions(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPoseBlue() {
            return pose;
        }

        // Mirror the pose for red alliance, by getting the x distance of the pose to the blue reef, and then adding
        // it to the blue reef x coordinate. The Y coordinate stays the same.
        public Pose2d getPoseRed() {
            return new Pose2d(
                    new Translation2d(Positions.Reef.redTranslation2d.getX() - (Positions.Reef.blueTranslation2d.getX() - pose.getX()),
                            pose.getY()),
                    pose.getRotation());
        }
    }
}
