package frc.robot.vision;

import org.photonvision.EstimatedRobotPose;

public class VisionPoseEstimationResult {
    private final VisionCamera visionCamera;
    private final EstimatedRobotPose estimatedRobotPose;

    public VisionPoseEstimationResult(VisionCamera visionCamera, EstimatedRobotPose estimatedRobotPose) {
        this.visionCamera = visionCamera;
        this.estimatedRobotPose = estimatedRobotPose;
    }

    public VisionCamera getVisionCamera() {
        return visionCamera;
    }

    public EstimatedRobotPose getEstimatedRobotPose() {
        return estimatedRobotPose;
    }
}
