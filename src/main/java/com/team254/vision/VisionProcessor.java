package com.team254.vision;

import static com.team841.dory.constants.RC.LOOKBACK_TIME;

import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import com.team254.lib.ConcurrentTimeInterpolatableBuffer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import dev.doglog.DogLog;

public class VisionProcessor{

    public double lastProcessedTimestamp;

    public final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer;
    
    public final ConcurrentTimeInterpolatableBuffer<Pose2d> robotPose =
            ConcurrentTimeInterpolatableBuffer.createBuffer(LOOKBACK_TIME);
    public final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());

    public final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    public final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    public final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());

    public double lastUsedMegatagTimestamp = 0;

    public ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    
    public VisionProcessor(Consumer<VisionFieldPoseEstimate> visionEstimateConsumer) {
        this.visionEstimateConsumer = visionEstimateConsumer;
        robotPose.addSample(0.0, Pose2d.kZero);
        driveYawAngularVelocity.addSample(0.0, 0.0);
    }

    public void updateVision(
                            boolean cameraSeesTarget,
                            FiducialObservation[] cameraFiducialObservations,
                            MegatagPoseEstimate cameraMegatagPoseEstimate,
                            MegatagPoseEstimate cameraMegatag2PoseEstimate,
                            String logPreface) {

        if (cameraMegatagPoseEstimate != null) {
            var updateTimestamp = cameraMegatagPoseEstimate.timestampSeconds;

            boolean alreadyProcessedTimestamp = lastProcessedTimestamp == updateTimestamp;

            if (!alreadyProcessedTimestamp && cameraSeesTarget) {

                Optional<VisionFieldPoseEstimate> megatagEstimate =
                        processMegatagPoseEstimate(cameraMegatagPoseEstimate);
                Optional<VisionFieldPoseEstimate> megatag2Estimate =
                        processMegatag2PoseEstimate(cameraMegatag2PoseEstimate, logPreface);

                boolean used_megatag = false;

                if (megatagEstimate.isPresent()) {
                    if (shouldUseMegatag(cameraMegatagPoseEstimate, cameraFiducialObservations, logPreface)) {
                        DogLog.log(
                                logPreface +
                                        "MegatagEstimate", megatagEstimate.get().getVisionRobotPoseMeters());
                        updateMegatagEstimate(megatagEstimate.get());
                        used_megatag = true;
                    } else {
                        DogLog.log(
                                logPreface +
                                        "MegatagEstimateRejected", megatagEstimate.get().getVisionRobotPoseMeters());
                    }
                }

                if (megatag2Estimate.isPresent() && !used_megatag) {
                    if (shouldUseMegatag2(cameraMegatag2PoseEstimate.timestampSeconds, logPreface)) {
                        DogLog.log(
                                logPreface + "Megatag2Estimate", megatag2Estimate.get().getVisionRobotPoseMeters());
                        updateMegatag2Estimate(megatag2Estimate.get());
                    } else {
                        if (megatagEstimate.isPresent()) {
                            DogLog.log(
                                    logPreface +
                                            "Megatag2EstimateRejected",
                                    megatag2Estimate.get().getVisionRobotPoseMeters());
                        }
                    }
                }

                lastProcessedTimestamp = updateTimestamp;
            }
        }
    }

    private Optional<VisionFieldPoseEstimate> processMegatagPoseEstimate(MegatagPoseEstimate poseEstimate) {
        var loggedFieldToRobot = getFieldToRobot(poseEstimate.timestampSeconds);
        if (loggedFieldToRobot.isEmpty()) {
            return Optional.empty();
        }

        var fieldToRobotEstimate = poseEstimate.fieldToCamera;

        // distance from current pose to vision estimated pose
        double poseDifference =
                fieldToRobotEstimate.getTranslation().getDistance(loggedFieldToRobot.get().getTranslation());

        if (poseEstimate.fiducialIds.length > 0) {
            double xyStds = 1.0;
            double degStds = 12;
            // multiple targets detected
            if (poseEstimate.fiducialIds.length >= 2) {
                xyStds = 0.5;
                degStds = 6;
            }
            // 1 target with large area and close to estimated pose
            else if (poseEstimate.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            }
            // 1 target farther away and estimated pose is close
            else if (poseEstimate.avgTagArea > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            }

            Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds));
            return Optional.of(new VisionFieldPoseEstimate(
                    fieldToRobotEstimate, poseEstimate.timestampSeconds, visionMeasurementStdDevs));
        }
        return Optional.empty();
    }

    private Optional<VisionFieldPoseEstimate> processMegatag2PoseEstimate(
                                                                        MegatagPoseEstimate poseEstimate,
                                                                        String logPreface) {
        var loggedFieldToRobot = getFieldToRobot(poseEstimate.timestampSeconds);
        if (loggedFieldToRobot.isEmpty()) {
            return Optional.empty();
        }

        var fieldToRobotEstimate = poseEstimate.fieldToCamera;

        // distance from current pose to vision estimated pose
        double poseDifference =
                fieldToRobotEstimate.getTranslation().getDistance(loggedFieldToRobot.get().getTranslation());

        double xyStds;
        if (poseEstimate.fiducialIds.length > 0) {
            // multiple targets detected
            if (poseEstimate.fiducialIds.length >= 2 && poseEstimate.avgTagArea > 0.1) {
                xyStds = 0.2;
            }

            // 1 target with large area and close to estimated pose
            else if (poseEstimate.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStds = 0.5;
            }
            // 1 target farther away and estimated pose is close
            else if (poseEstimate.avgTagArea > 0.1 && poseDifference < 0.3) {
                xyStds = 1.0;
            } else if (poseEstimate.fiducialIds.length > 1) {
                xyStds = 1.2;
            } else {
                xyStds = 2.0;
            }

            DogLog.log(logPreface + "Megatag2StdDev", xyStds);
            DogLog.log(logPreface + "Megatag2AvgTagArea", poseEstimate.avgTagArea);
            DogLog.log(logPreface + "Megatag2PoseDifference", poseDifference);

            Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(50.0));
            fieldToRobotEstimate = new Pose2d(
                    fieldToRobotEstimate.getTranslation(), loggedFieldToRobot.get().getRotation());
            return Optional.of(new VisionFieldPoseEstimate(
                    fieldToRobotEstimate, poseEstimate.timestampSeconds, visionMeasurementStdDevs));
        }
        return Optional.empty();
    }

    private boolean shouldUseMegatag(
                                    MegatagPoseEstimate poseEstimate,
                                    FiducialObservation[] fiducials,
                                    String logPreface) {
        final double kMinAreaForMegatagEnabled = 0.4;
        final double kMinAreaForMegatagDisabled = 0.05;

        double kMinAreaForMegatag = 0.0;
        if (DriverStation.isDisabled()) {
            kMinAreaForMegatag = kMinAreaForMegatagDisabled;
        } else {
            kMinAreaForMegatag = kMinAreaForMegatagEnabled;
        }

        final int kExpectedTagCount = 2;

        final double kLargeYawThreshold = Units.degreesToRadians(200.0);
        final double kLargeYawEventTimeWindowS = 0.05;

        var maxYawVel = getMaxAbsDriveYawAngularVelocityInRange(
                poseEstimate.timestampSeconds - kLargeYawEventTimeWindowS, poseEstimate.timestampSeconds);
        if (maxYawVel.isPresent() && Math.abs(maxYawVel.get()) > kLargeYawThreshold) {
            DogLog.log("Vision/MegatagYawAngular", false);
            return false;
        }
        DogLog.log("Vision/MegatagYawAngular", true);

        if (poseEstimate.avgTagArea < kMinAreaForMegatag) {
            DogLog.log(logPreface + "megaTagAvgTagArea", false);
            return false;
        }
        DogLog.log(logPreface + "megaTagAvgTagArea", true);

        if (poseEstimate.fiducialIds.length != kExpectedTagCount) {
            DogLog.log(logPreface + "fiducialLength", false);
            return false;
        }
        DogLog.log(logPreface + "fiducialLength", true);

        if (poseEstimate.fiducialIds.length < 1) {
            DogLog.log(logPreface + "fiducialLengthLess1", false);
            return false;
        }
        DogLog.log(logPreface + "fiducialLengthLess1", true);

        if (poseEstimate.fieldToCamera.getTranslation().getNorm() < 1.0) {
            DogLog.log(logPreface + "NormCheck", false);
            return false;
        }
        DogLog.log(logPreface + "NormCheck", true);

        for (var fiducial : fiducials) {
            if (fiducial.ambiguity > .9) {
                DogLog.log(logPreface + "Ambiguity", false);
                return false;
            }
        }
        DogLog.log(logPreface + "Ambiguity", true);

        return true;
    }

    private boolean shouldUseMegatag2(double timestamp, String preface) {
        final double kLargePitchRollYawEventTimeWindowS = 0.1;
        final double kLargeYawThreshold = Units.degreesToRadians(100.0);

        var maxYawVel =
                getMaxAbsDriveYawAngularVelocityInRange(timestamp - kLargePitchRollYawEventTimeWindowS, timestamp);
        if (maxYawVel.isPresent() && Math.abs(maxYawVel.get()) > kLargeYawThreshold) {
            DogLog.log(preface + "PinholeYawAngular", false);
            return false;
        }
        DogLog.log(preface + "PinholeYawAngular", true);

        return true;
    }

    public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
        return robotPose.getLatest();
    }

    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return robotPose.getSample(timestamp);
    }

    private Optional<Double> getMaxAbsValueInRange(
                                                ConcurrentTimeInterpolatableBuffer<Double> buffer,
                                                double minTime,
                                                double maxTime) {
        var submap = buffer.getInternalBuffer().subMap(minTime, maxTime).values();
        var max = submap.stream().max(Double::compare);
        var min = submap.stream().min(Double::compare);
        if (max.isEmpty() || min.isEmpty()) return Optional.empty();
        if (Math.abs(max.get()) >= Math.abs(min.get())) return max;
        else return min;
    }

    public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
        return measuredRobotRelativeChassisSpeeds.get();
    }

    public Optional<Double> getMaxAbsDriveYawAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsValueInRange(driveYawAngularVelocity, minTime, maxTime);
    }

    public void updateMegatagEstimate(VisionFieldPoseEstimate megatagEstimate) {
        lastUsedMegatagTimestamp = Timer.getTimestamp();
        visionEstimateConsumer.accept(megatagEstimate);
    }

    public void updateMegatag2Estimate(VisionFieldPoseEstimate megatagEstimate) {
        visionEstimateConsumer.accept(megatagEstimate);
    }
}