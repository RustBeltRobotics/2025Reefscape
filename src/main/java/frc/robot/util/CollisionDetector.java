package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.hardware.MultiGyro;

public class CollisionDetector {

    private final MultiGyro gyro;
    private final Alert collisionDetectedAlert = new Alert("Collision detected!", AlertType.kWarning);
    private double currentLinearAccelerationX;
    private double currentLinearAccelerationY;
    private double lastLinearAccelerationX;
    private double lastLinearAccelerationY;
    private long collisionTimeMicroSeconds;
    private boolean collisionDetected;

    public CollisionDetector(MultiGyro gyro) {
        this.gyro = gyro;
        currentLinearAccelerationX = gyro.getAccelerationX();
        currentLinearAccelerationY = gyro.getAccelerationY();
        lastLinearAccelerationX = currentLinearAccelerationX;
        lastLinearAccelerationY = currentLinearAccelerationY;
    }

    public boolean checkForCollision() {
        lastLinearAccelerationX = currentLinearAccelerationX;
        lastLinearAccelerationY = currentLinearAccelerationY;
        currentLinearAccelerationX = gyro.getAccelerationX();
        currentLinearAccelerationY = gyro.getAccelerationY();
        double currentJerkX = Math.abs(currentLinearAccelerationX - lastLinearAccelerationX);
        double currentJerkY = Math.abs(currentLinearAccelerationY - lastLinearAccelerationY);
        double collisionThreshold = Constants.Kinematics.COLLISION_THRESHOLD_DELTA_G_TELE_OP;

        if (DriverStation.isAutonomous()) {
            collisionThreshold = Constants.Kinematics.COLLISION_THRESHOLD_DELTA_G_AUTONOMOUS;
        }

        if (currentJerkX > collisionThreshold || currentJerkY > collisionThreshold) {
            collisionDetected = true;
            collisionDetectedAlert.set(true);
            collisionTimeMicroSeconds = RobotController.getFPGATime();
            DataLogManager.log("RBR: Collision detected!");
        }

        return collisionDetected;
    }

    public boolean isStabilized() {
        long currentTimeMicroSeconds = RobotController.getFPGATime();
        long timeSinceCollision = currentTimeMicroSeconds - collisionTimeMicroSeconds;
        
        return timeSinceCollision >= Constants.Kinematics.MICROSECONDS_SINCE_COLLISION_THRESHOLD;
    }

    public void clearDetectedCollision() {
        if (this.collisionDetected) {
            collisionDetected = false;
            collisionDetectedAlert.set(false);
            DataLogManager.log("RBR: Detected collision cleared");
        }
    }

    public boolean isCollisionDetected() {
        return collisionDetected;
    }

    public double getCurrentLinearAccelerationX() {
        return currentLinearAccelerationX;
    }

    public void setCurrentLinearAccelerationX(double currentLinearAccelerationX) {
        this.currentLinearAccelerationX = currentLinearAccelerationX;
    }

    public double getCurrentLinearAccelerationY() {
        return currentLinearAccelerationY;
    }

    public void setCurrentLinearAccelerationY(double currentLinearAccelerationY) {
        this.currentLinearAccelerationY = currentLinearAccelerationY;
    }
    
}
