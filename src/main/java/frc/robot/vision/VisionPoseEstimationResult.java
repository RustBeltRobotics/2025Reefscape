package frc.robot.vision;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionPoseEstimationResult {
    private final VisionCamera visionCamera;
    private final EstimatedRobotPose estimatedRobotPose;
    private final Matrix<N3, N1> visionMeasurementStdDevs;

    public VisionPoseEstimationResult(VisionCamera visionCamera, EstimatedRobotPose estimatedRobotPose, Matrix<N3, N1> visionMeasurementStdDevs) {
        this.visionCamera = visionCamera;
        this.estimatedRobotPose = estimatedRobotPose;
        this.visionMeasurementStdDevs = visionMeasurementStdDevs;
    }

    public VisionCamera getVisionCamera() {
        return visionCamera;
    }

    public EstimatedRobotPose getEstimatedRobotPose() {
        return estimatedRobotPose;
    }

    public Matrix<N3, N1> getVisionMeasurementStdDevs() {
        return visionMeasurementStdDevs;
    }

}
