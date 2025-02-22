package frc.robot.vision;

import static frc.robot.constants.FieldConstants.aprilTagFieldLayout;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.GeomUtil;
import frc.robot.commons.PolynomialRegression;
import frc.robot.commons.TimestampedVisionUpdate;

//import frc.robot.constants.FieldConstants;

import frc.robot.FieldConstants;
//new and improved
//import frc.robot.constants.SingleTagAdjusters;

import frc.robot.constants.SingleTagAdjusters;
// ** needs to be brought in


import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
//replacing littleton robotics with 424 common import
//import org.littletonrobotics.junction.Logger;
//might not be used or is renamed
// what is that??
//   https://github.com/Mechanical-Advantage/RobotCode2025Public



import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class GlobalAprilTagVision extends SubsystemBase {
  private PhotonCamera[] cameras;
  private static final double fieldBorderMargin = 0.5;
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};
  private List<TimestampedVisionUpdate> visionUpdates;
  private Supplier<Pose2d> poseSupplier = () -> new Pose2d();

  private double stdDevScalar = 0.2;
  private double thetaStdDevCoefficient = 0.075;

  private PolynomialRegression xyStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.12, 0.14, 0.17, 0.27, 0.38},
          2);
  private PolynomialRegression thetaStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029, 0.068},
          1);

  private final Pose3d[] cameraPoses =
      new Pose3d[] {
        // Front Left
        new Pose3d(
            Units.inchesToMeters(6.67),
            Units.inchesToMeters(12.74),
            Units.inchesToMeters(25.1),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-25.0),
                Units.degreesToRadians(15.0))),
        // Front Right
        new Pose3d(
            Units.inchesToMeters(6.67),
            Units.inchesToMeters(-12.74),
            Units.inchesToMeters(25.1),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-25.0),
                Units.degreesToRadians(-15.0))),
        // Back Left
        new Pose3d(
            Units.inchesToMeters(1.33),
            Units.inchesToMeters(12.74),
            Units.inchesToMeters(25.1),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-25.0),
                Units.degreesToRadians(165.0))),
        // Back Right
        new Pose3d(
            Units.inchesToMeters(1.33),
            Units.inchesToMeters(-12.74),
            Units.inchesToMeters(25.1),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-25.0),
                Units.degreesToRadians(-165.0))),
      };

  public GlobalAprilTagVision(PhotonCamera... cameras) {
    this.cameras = cameras;
  }

  public void setDataInterfaces(
      Supplier<Pose2d> poseSupplier, Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    this.poseSupplier = poseSupplier;
    this.visionConsumer = visionConsumer;
  }

  @Override
  public void periodic() {
    Pose2d currentPose = poseSupplier.get();
    visionUpdates = new ArrayList<>();

    // Loop through all the cameras
    for (int instanceIndex = 0; instanceIndex < cameras.length; instanceIndex++) {

      // Camera-specific variables
      Pose3d cameraPose;
      Pose2d robotPose;
      List<Pose3d> tagPose3ds = new ArrayList<>();
      List<PhotonPipelineResult> unprocessedResults = cameras[instanceIndex].getAllUnreadResults();

      double singleTagAdjustment = 1.0;

      DataLogManager.recordOutput(
      //Logger.recordOutput(
          "Photon/Camera " + instanceIndex + "ResultsLength", unprocessedResults.size());

      // Continue if camera hasn't processed a new frame since last check
      if (unprocessedResults.isEmpty()) {
        continue;
      }

      // Filter through each unread pipeline result and add to pose estimator with corresponding
      // timestamp
      for (PhotonPipelineResult unprocessedResult : unprocessedResults) {
        
        DataLogManager.recordOutput(
        //Logger.recordOutput(
            "Photon/Camera " + instanceIndex + " Has Targets", unprocessedResult.hasTargets());
        DataLogManager.recordOutput(
        //Logger.recordOutput(
            "Photon/Camera " + instanceIndex + "LatencyMS",
            unprocessedResult.metadata.getLatencyMillis());
            
        DataLogManager.recordOutput(
        //Logger.recordOutput(
            "Photon/Raw Camera Data " + instanceIndex,
            SmartDashboard.getRaw(
                "photonvision/" + cameras[instanceIndex].getName() + "/rawBytes", new byte[] {}));

        // Continue if the camera doesn't have any targets
        if (!unprocessedResult.hasTargets()) {
          DataLogManager.recordOutput("Photon/Tags Used " + instanceIndex, 0);
          //Logger.recordOutput("Photon/Tags Used " + instanceIndex, 0);
          continue;
        }

        double timestamp = unprocessedResult.getTimestampSeconds();
        DataLogManager.recordOutput("Photon/Camera " + instanceIndex + " Timestamp", timestamp);
        //Logger.recordOutput("Photon/Camera " + instanceIndex + " Timestamp", timestamp);

        boolean shouldUseMultiTag = unprocessedResult.getMultiTagResult().isPresent();

        if (shouldUseMultiTag) {
          // If multitag, use directly
          cameraPose =
              GeomUtil.transform3dToPose3d(
                  unprocessedResult.getMultiTagResult().get().estimatedPose.best);

          robotPose =
              cameraPose
                  .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                  .toPose2d();

          // Populate array of tag poses with tags used
          for (int id : unprocessedResult.getMultiTagResult().get().fiducialIDsUsed) {
            tagPose3ds.add(aprilTagFieldLayout.getTagPose(id).get());
          }
          DataLogManager.recordOutput("Photon/Camera Pose (Multi tag) " + instanceIndex, cameraPose);
          //Logger.recordOutput("Photon/Camera Pose (Multi tag) " + instanceIndex, cameraPose);
        } else {
          // If not using multitag, disambiugate and then use
          PhotonTrackedTarget target = unprocessedResult.targets.get(0);

          // if (!(target.getFiducialId() == 3
          //     || target.getFiducialId() == 4
          //     || target.getFiducialId() == 7
          //     || target.getFiducialId() == 8)) {
          //   continue;
          // }

          if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isEmpty()) {
            continue;
          }

          Pose3d tagPos = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();

          Pose3d cameraPose0 = tagPos.transformBy(target.getBestCameraToTarget().inverse());
          Pose3d cameraPose1 = tagPos.transformBy(target.getAlternateCameraToTarget().inverse());
          Pose2d robotPose0 =
              cameraPose0
                  .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                  .toPose2d();
          Pose2d robotPose1 =
              cameraPose1
                  .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                  .toPose2d();

          double projectionError = target.getPoseAmbiguity();

          // Select a pose using projection error and current rotation
          if (projectionError < 0.15) {
            cameraPose = cameraPose0;
            robotPose = robotPose0;
          } else if (Math.abs(
                  robotPose0.getRotation().minus(currentPose.getRotation()).getRadians())
              < Math.abs(robotPose1.getRotation().minus(currentPose.getRotation()).getRadians())) {
            cameraPose = cameraPose0;
            robotPose = robotPose0;
          } else {
            cameraPose = cameraPose1;
            robotPose = robotPose1;
          }

          tagPose3ds.add(tagPos);
          singleTagAdjustment = SingleTagAdjusters.getAdjustmentForTag(target.getFiducialId());
          DataLogManager.recordOutput("Photon/Camera Pose (Single Tag) " + instanceIndex, cameraPose);
          //Logger.recordOutput("Photon/Camera Pose (Single Tag) " + instanceIndex, cameraPose);
        }

        if (cameraPose == null || robotPose == null) {
          continue;
        }

        // Move on to next camera if robot pose is off the field
        if (robotPose.getX() < -fieldBorderMargin
            || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
            || robotPose.getY() < -fieldBorderMargin
            || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin) {
          continue;
        }

        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPose3ds) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        double avgDistance = totalDistance / tagPose3ds.size();
        double xyStdDev = 0.0;
        double thetaStdDev = 0.0;

        if (shouldUseMultiTag) {
          xyStdDev = Math.pow(avgDistance, 2.0) / tagPose3ds.size();
          thetaStdDev = Math.pow(avgDistance, 2.0) / tagPose3ds.size();
        } else {
          xyStdDev = xyStdDevModel.predict(avgDistance);
          thetaStdDev = thetaStdDevModel.predict(avgDistance);
        }

        if (shouldUseMultiTag) {
          visionUpdates.add(
              new TimestampedVisionUpdate(
                  robotPose,
                  timestamp,
                  VecBuilder.fill(
                      stdDevScalar * thetaStdDevCoefficient * xyStdDev,
                      stdDevScalar * thetaStdDevCoefficient * xyStdDev,
                      stdDevScalar * thetaStdDevCoefficient * thetaStdDev)));
        } else {
          visionUpdates.add(
              new TimestampedVisionUpdate(
                  robotPose,
                  timestamp,
                  VecBuilder.fill(
                      singleTagAdjustment * xyStdDev * stdDevScalar,
                      singleTagAdjustment * xyStdDev * stdDevScalar,
                      singleTagAdjustment * thetaStdDev * stdDevScalar)));
          
          DataLogManager.recordOutput("VisionData/" + instanceIndex, robotPose);
          //Logger.recordOutput("VisionData/" + instanceIndex, robotPose);
          DataLogManager.recordOutput("Photon/Tags Used " + instanceIndex, tagPose3ds.size());
          //Logger.recordOutput("Photon/Tags Used " + instanceIndex, tagPose3ds.size());
        }
      }
    }

    // Apply all vision updates to pose estimator
    visionConsumer.accept(visionUpdates);
  }
}