package frc.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionCamera {

    private final String cameraName;
    private final PhotonCamera cameraInstance;
    private final PhotonPoseEstimator poseEstimator;
    private final Transform3d robotToCameraPose;
    private final CameraPosition cameraPosition;

    public VisionCamera(String cameraName, CameraPosition cameraPosition, Transform3d robotToCameraPose, PoseStrategy poseStrategy, AprilTagFieldLayout fieldLayout) {
        this.cameraName = cameraName;
        this.cameraPosition = cameraPosition;
        this.robotToCameraPose = robotToCameraPose;
        cameraInstance = new PhotonCamera(cameraName);
        poseEstimator = new PhotonPoseEstimator(fieldLayout, poseStrategy, robotToCameraPose);
    }

    public String getCameraName() {
        return cameraName;
    }

    public PhotonCamera getCameraInstance() {
        return cameraInstance;
    }

    public PhotonPoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public Transform3d getRobotToCameraPose() {
        return robotToCameraPose;
    }

    public CameraPosition getCameraPosition() {
        return cameraPosition;
    }

    public boolean isCameraConnected() {
        return cameraInstance.isConnected();
    }
}
