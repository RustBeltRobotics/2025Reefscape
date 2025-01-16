package frc.robot.util;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class CollisionDetector {

    private final Pigeon2 gyro;
    private final Alert collisionDetectedAlert = new Alert("Collision detected!", AlertType.kWarning);
    private double currentLinearAccelerationX;
    private double currentLinearAccelerationY;
    private double lastLinearAccelerationX;
    private double lastLinearAccelerationY;
    private long collisionTimeMicroSeconds;
    private boolean collisionDetected;

    public CollisionDetector(Pigeon2 gyro) {
        this.gyro = gyro;
        currentLinearAccelerationX = gyro.getAccelerationX().getValueAsDouble();
        currentLinearAccelerationY = gyro.getAccelerationY().getValueAsDouble();
        lastLinearAccelerationX = currentLinearAccelerationX;
        lastLinearAccelerationY = currentLinearAccelerationY;
    }

    public boolean checkForCollision() {
        lastLinearAccelerationX = currentLinearAccelerationX;
        lastLinearAccelerationY = currentLinearAccelerationY;
        currentLinearAccelerationX = gyro.getAccelerationX().getValueAsDouble();
        currentLinearAccelerationY = gyro.getAccelerationY().getValueAsDouble();
        double currentJerkX = Math.abs(currentLinearAccelerationX - lastLinearAccelerationX);
        double currentJerkY = Math.abs(currentLinearAccelerationY - lastLinearAccelerationY);

        if (currentJerkX > Constants.Kinematics.COLLISION_THRESHOLD_DELTA_G || currentJerkY > Constants.Kinematics.COLLISION_THRESHOLD_DELTA_G) {
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
