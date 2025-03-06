package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants;
import frc.robot.util.AlertManager;
import frc.robot.vision.CameraPosition;
import frc.robot.vision.VisionCamera;
import frc.robot.vision.VisionPoseEstimationResult;

//TODO: MJR review latest photonvision example for 2025 season and update logic below accordingly
// https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java

/**
 * Photonvision based vision system - note we are NOT modeled as a Subsystem, since we don't want to block any commands
 * 
 */
public class VisionSystem {

    private final AprilTagFieldLayout fieldLayout;
    private final List<VisionCamera> visionCameras = new ArrayList<>();
    private final StructArrayPublisher<Pose3d> acceptedTagPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/RBR/Vision/AprilTags/Accepted", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> rejectedTagPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/RBR/Vision/AprilTags/Rejected", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> acceptedVisionPosePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/RBR/Vision/PoseEstimates/Accepted", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> rejectedVisionPosePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/RBR/Vision/PoseEstimates/Rejected", Pose3d.struct).publish();
    private final StructPublisher<Pose2d> frontCenterCameraPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/RBR/Vision/PoseEstimates/Camera/FC", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> frontRightCameraPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/RBR/Vision/PoseEstimates/Camera/FR", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> backLeftCameraPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/RBR/Vision/PoseEstimates/Camera/BL", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> backRightCameraPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/RBR/Vision/PoseEstimates/Camera/BR", Pose2d.struct).publish();

    //Links for troubleshooting / understanding:
    /*
     * https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/index.html
     * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/pose.html
     * https://www.chiefdelphi.com/t/trouble-with-cameratotarget-and-robot-pose-from-photonvision/454206
     * https://github.com/TexasTorque/TorqueLib/blob/master/sensors/TorqueVision.java
     */

    public VisionSystem() {
        //TODO: if the venue is not using the default welded field layout and is instead using AndyMark, update this to AprilTagFields.k2025ReefscapeAndyMark
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        VisionCamera frontCenterCamera = new VisionCamera(Constants.Vision.CameraName.FRONT_CENTER, CameraPosition.FRONT_CENTER,
            Constants.Vision.CameraPose.FRONT_CENTER, Constants.Vision.POSE_STRATEGY, fieldLayout);   
        // VisionCamera frontRightCamera = new VisionCamera(Constants.Vision.CameraName.FRONT_RIGHT, CameraPosition.FRONT_RIGHT,
        //     Constants.Vision.CameraPose.FRONT_RIGHT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, fieldLayout);
        VisionCamera backRightCamera = new VisionCamera(Constants.Vision.CameraName.BACK_RIGHT, CameraPosition.BACK_RIGHT,
            Constants.Vision.CameraPose.BACK_RIGHT, Constants.Vision.POSE_STRATEGY, fieldLayout);
        VisionCamera backLeftCamera = new VisionCamera(Constants.Vision.CameraName.BACK_LEFT, CameraPosition.BACK_LEFT,
            Constants.Vision.CameraPose.BACK_LEFT, Constants.Vision.POSE_STRATEGY, fieldLayout);
        visionCameras.add(frontCenterCamera);           
        // visionCameras.add(frontRightCamera);           
        visionCameras.add(backRightCamera);           
        visionCameras.add(backLeftCamera);

        //Do not track cameras that are not actively connected at time of constructor intiialization
        visionCameras.removeIf(c -> {
            boolean isDisconnected = !c.isCameraConnected();
            if (isDisconnected) {
                AlertManager.addAlert(c.getCameraName(), "Camera (" + c.getCameraName() + ") disconnected!", AlertType.kWarning);
            } else {
                AlertManager.removeAlert(c.getCameraName());
                c.getCameraInstance().setPipelineIndex(Constants.Vision.APRIL_TAG_PIPELINE_INDEX);
            }

            return isDisconnected;
        });
    }
 
    /**
     * Get position estimates of robot based on vision data (April Tag readings)
     * 
     * @return List of VisionPoseEstimationResult - one per camera that found a pose estimate
     */
    public List<VisionPoseEstimationResult> getRobotPoseEstimationResults() {
        List<VisionPoseEstimationResult> estimationResults = new ArrayList<>(visionCameras.size());
        List<EstimatedRobotPose> acceptedPoses = new ArrayList<>(visionCameras.size());
        List<EstimatedRobotPose> rejectedPoses = new ArrayList<>(visionCameras.size());
        List<Pose3d> usedAprilTags = new ArrayList<>();
        List<Pose3d> rejectedAprilTags = new ArrayList<>();

        for (int i = 0; i < visionCameras.size(); i++) {
            VisionCamera visionCamera = visionCameras.get(i);
            PhotonCamera photonCamera = visionCamera.getCameraInstance();
            PhotonPoseEstimator poseEstimator = visionCamera.getPoseEstimator();

            for (PhotonPipelineResult pipelineResult : photonCamera.getAllUnreadResults()) {
                processPhotonPipelineResult(pipelineResult, estimationResults, acceptedPoses, rejectedPoses, usedAprilTags,
                    rejectedAprilTags, visionCamera, poseEstimator);
            }
        }

        if (!usedAprilTags.isEmpty()) {
            acceptedTagPublisher.accept(usedAprilTags.toArray(new Pose3d[0]));
        }
        if (!rejectedAprilTags.isEmpty()) {
            rejectedTagPublisher.accept(rejectedAprilTags.toArray(new Pose3d[0]));
        }

        if (!acceptedPoses.isEmpty()) {
            List<Pose3d> acceptedPoses3dList = new ArrayList<>();
            for (EstimatedRobotPose pose : acceptedPoses) {
                acceptedPoses3dList.add(pose.estimatedPose);
            }
            acceptedVisionPosePublisher.accept(acceptedPoses3dList.toArray(new Pose3d[0]));
        }
        if (!rejectedPoses.isEmpty()) {
            List<Pose3d> rejectedPoses3dList = new ArrayList<>();
            for (EstimatedRobotPose pose : rejectedPoses) {
                rejectedPoses3dList.add(pose.estimatedPose);
            }
            rejectedVisionPosePublisher.accept(rejectedPoses3dList.toArray(new Pose3d[0]));
        }

        return estimationResults;
    }

    private void processPhotonPipelineResult(PhotonPipelineResult pipelineResult, List<VisionPoseEstimationResult> estimationResults, 
            List<EstimatedRobotPose> acceptedPoses, List<EstimatedRobotPose> rejectedPoses, 
            List<Pose3d> usedAprilTags, List<Pose3d> rejectedAprilTags, VisionCamera visionCamera, PhotonPoseEstimator poseEstimator) {
        List<Pose3d> usedAprilTagsForCamera = new ArrayList<>();
        List<Pose3d> rejectedAprilTagsForCamera = new ArrayList<>();
        Optional<MultiTargetPNPResult> multiTagResult = pipelineResult.getMultiTagResult();

        //https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html#enabling-multitag
        if (multiTagResult.isPresent()) {
            MultiTargetPNPResult mtr = multiTagResult.get();
            Transform3d fieldToCamera = mtr.estimatedPose.best;
            //TODO: discard poseEstimateResult below if bestReprojectionError is too high
            double bestReprojectionError = mtr.estimatedPose.bestReprojErr;  //uom is pixels
            //TODO: compare this with the EstimatedRobotPose result from below
            Pose3d multiTagEstimatedRobotPose = new Pose3d(fieldToCamera.getX(), fieldToCamera.getY(), fieldToCamera.getZ(), fieldToCamera.getRotation());
        }

        //TODO: verify this is non-empty when running the AprilTag pipeline and not object detection pipeline
        //  This whole block may be unnecessary due to poseEstimator.update(pipelineResult) below already taking multitag logic into account
        if (pipelineResult.hasTargets()) {
            int numAprilTagsSeen = pipelineResult.getTargets().size();

            for (PhotonTrackedTarget target : pipelineResult.getTargets()) {
                //poseAmbiguity is between 0 and 1 (0 being no ambiguity, and 1 meaning both have the same reprojection error). Numbers above 0.2 are likely to be ambiguous. -1 if invalid.
                double poseAmbiguity = target.getPoseAmbiguity();
                boolean isUnambiguous = poseAmbiguity < Constants.Vision.POSE_AMBIGUITY_CUTOFF && poseAmbiguity >= 0;
                boolean isCloseEnough = target.getBestCameraToTarget().getTranslation().getNorm() < Constants.Vision.DISTANCE_CUTOFF;
                boolean shouldUseTarget = (isUnambiguous || numAprilTagsSeen > 1) && isCloseEnough;
                Optional<Pose3d> targetPose = fieldLayout.getTagPose(target.getFiducialId());
                if (targetPose.isPresent()) {
                    Pose3d tagPose = targetPose.get();

                    if (shouldUseTarget) {
                        usedAprilTagsForCamera.add(tagPose);
                    } else {
                        rejectedAprilTagsForCamera.add(tagPose);
                    }
                }
            }

            if (!usedAprilTagsForCamera.isEmpty()) {
                usedAprilTags.addAll(usedAprilTagsForCamera);
            }
            if (!rejectedAprilTagsForCamera.isEmpty()) {
                rejectedAprilTags.addAll(rejectedAprilTagsForCamera);
            }
        }

        Optional<EstimatedRobotPose> poseEstimateResult = poseEstimator.update(pipelineResult);
        if (poseEstimateResult.isPresent()) {
            EstimatedRobotPose poseEstimate = poseEstimateResult.get();
            double visionPoseTimestampSeconds = poseEstimate.timestampSeconds;
            long networkTablesPoseTimestampMicroSeconds = Math.round(visionPoseTimestampSeconds * 1000000); //microseconds
            Pose3d estimatedPose = poseEstimate.estimatedPose;
            Pose2d estimatedPose2d = estimatedPose.toPose2d();

            if (!usedAprilTagsForCamera.isEmpty()) {
                // Do not use pose if robot pose is off the field 
                if (estimatedPose.getX() < -Constants.Game.FIELD_POSE_XY_ERROR_MARGIN_METERS
                    || estimatedPose.getX() > fieldLayout.getFieldLength() + Constants.Game.FIELD_POSE_XY_ERROR_MARGIN_METERS
                    || estimatedPose.getY() < -Constants.Game.FIELD_POSE_XY_ERROR_MARGIN_METERS
                    || estimatedPose.getY() > fieldLayout.getFieldWidth() + Constants.Game.FIELD_POSE_XY_ERROR_MARGIN_METERS) {
                        rejectedPoses.add(poseEstimate);
                        return;
                }

                acceptedPoses.add(poseEstimate);
                estimationResults.add(new VisionPoseEstimationResult(visionCamera, poseEstimate, getVisionMeasurementStandardDeviation(poseEstimate)));

                if (visionCamera.getCameraPosition() == CameraPosition.FRONT_CENTER) {
                    frontCenterCameraPosePublisher.set(estimatedPose2d, networkTablesPoseTimestampMicroSeconds);
                } else if (visionCamera.getCameraPosition() == CameraPosition.FRONT_RIGHT) {
                    frontRightCameraPosePublisher.set(estimatedPose2d, networkTablesPoseTimestampMicroSeconds);
                } else if (visionCamera.getCameraPosition() == CameraPosition.BACK_LEFT) {
                    backLeftCameraPosePublisher.set(estimatedPose2d, networkTablesPoseTimestampMicroSeconds);
                } else if (visionCamera.getCameraPosition() == CameraPosition.BACK_RIGHT) {
                    backRightCameraPosePublisher.set(estimatedPose2d, networkTablesPoseTimestampMicroSeconds);
                }
            } else {
                rejectedPoses.add(poseEstimate);
            }
        }
    }

    public Matrix<N3, N1> getVisionMeasurementStandardDeviation(EstimatedRobotPose estimation) {
        double smallestTargetDistance = Double.POSITIVE_INFINITY;  //in meters
        double smallestPoseAmbiguity = Double.POSITIVE_INFINITY;
        int numTargetsUsed = estimation.targetsUsed.size();
        boolean singleTarget = numTargetsUsed == 1;
        double poseAmbiguityFactor = 1;  //default is for the case of multiple targets

        for (PhotonTrackedTarget target : estimation.targetsUsed) {
            //x = forward, y = left, z = up - from the camera
            Transform3d transform3d = target.getBestCameraToTarget();
            double poseAmbiguity = target.getPoseAmbiguity();
            double distance = Math.sqrt(Math.pow(transform3d.getX(), 2) + Math.pow(transform3d.getY(), 2) + Math.pow(transform3d.getZ(), 2));
            if (distance < smallestTargetDistance) {
                smallestTargetDistance = distance;
            }
            if (poseAmbiguity < smallestPoseAmbiguity) {
                smallestPoseAmbiguity = poseAmbiguity;
            }
        }

        if (singleTarget) {
            double ambiguityScore = smallestPoseAmbiguity + Constants.Vision.POSE_AMBIGUITY_SHIFTER;
            poseAmbiguityFactor = Math.max(1, ambiguityScore * Constants.Vision.POSE_AMBIGUITY_MULTIPLIER);
            double targetDistanceAdjusted = Math.max(0, smallestTargetDistance - Constants.Vision.NOISY_DISTANCE_METERS);
            double weightedDistance = Math.max(1, targetDistanceAdjusted * Constants.Vision.DISTANCE_WEIGHT); //weighted distance (further distances will be trusted less than shorter)
            double targetScore = weightedDistance * poseAmbiguityFactor;
            double weightedTagPresence = (1 + ((numTargetsUsed - 1) * Constants.Vision.TAG_PRESENCE_WEIGHT));  //more targets = more confidence
            double confidenceMultiplier = Math.max(1, targetScore / weightedTagPresence);

            return Constants.Vision.DEFAULT_VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
        }

        //Note: higher values = trust the vision measurement less
        double xyStdDev = Constants.Vision.MULTI_TAG_XY_PER_TAG_STANDARD_DEVIATION * numTargetsUsed;
        double thetaStdDev = Constants.Vision.MULTI_TAG_THETA_STANDARD_DEVIATION;
        if (smallestPoseAmbiguity >= 0.05) {
            xyStdDev += smallestPoseAmbiguity;
            thetaStdDev += smallestPoseAmbiguity;
        }
        
        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    public void takeRawImageSnapshot() {
        for (VisionCamera visionCamera : visionCameras) {
            if (visionCamera.isCameraConnected()) {
                visionCamera.getCameraInstance().takeInputSnapshot();
            }
        }
    }

    public void takeImageSnapshot() {
        for (VisionCamera visionCamera : visionCameras) {
            if (visionCamera.isCameraConnected()) {
                visionCamera.getCameraInstance().takeOutputSnapshot();
            }
        }
    }
}