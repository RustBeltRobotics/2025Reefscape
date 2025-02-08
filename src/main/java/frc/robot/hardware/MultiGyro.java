package frc.robot.hardware;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;

/**
 * Helper for utilizing multiple gyros (for redundancy)
 */
public class MultiGyro {

    // Pigeon2 connected over CAN
    private final Pigeon2 primary;
    private final AHRS secondary;  //navx
    private final Alert primaryInUseAlert = new Alert("Primary gyro (pigeon2) in use", AlertType.kInfo);
    private final Alert secondaryInUseAlert = new Alert("Secondary gyro (navx) in use", AlertType.kInfo);
    private boolean primaryIsActive = true;
    private boolean secondaryIsActive = false;

    public MultiGyro() {
        // Initialize and zero gyro
        primary = new Pigeon2(Constants.CanID.PIGEON_GYRO);
        primary.getConfigurator().apply(new Pigeon2Configuration());
        primary.setYaw(0);

        primaryIsActive = primary.isConnected();

        //navx auto-zeros after calibration completes
        secondary = new AHRS(AHRS.NavXComType.kMXP_SPI);
        if (!primaryIsActive) {
            secondaryIsActive = secondary.isConnected();
        }
        if (!secondaryIsActive) {
        }
        if (!primaryIsActive && !secondaryIsActive) {
            throw new RuntimeException("RBR: No gyros are connected!");
        }
        if (primaryIsActive) {
            primaryInUseAlert.set(true);
            DataLogManager.log("RBR: Gyro - using Pigeon2 as primary gyro");
        } else {
            secondaryInUseAlert.set(true);
            DataLogManager.log("RBR: Gyro - using navX as primary gyro");
        }
    }

    public double getYaw() {
        if (primaryIsActive) {
            return primary.getYaw().getValueAsDouble();
        } else if (secondaryIsActive) {
             //Note: we need to flip the sign since navx reports clockwise as positive, where wpilib/FRC coordinate system is counter-clockwise as positive
            return -(secondary.getYaw());
        } else {
            return 0;
        }
    }

    public Rotation2d getYawRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public double getPitch() {
        if (primaryIsActive) {
            return primary.getPitch().getValueAsDouble();
        } else if (secondaryIsActive) {
            return secondary.getPitch();
        } else {
            return 0;
        }
    }

    public double getRoll() {
        if (primaryIsActive) {
            return primary.getRoll().getValueAsDouble();
        } else if (secondaryIsActive) {
            return secondary.getRoll();
        } else {
            return 0;
        }
    }

    public double getAccelerationX() {
        if (primaryIsActive) {
            return primary.getAccelerationX().getValueAsDouble();
        } else if (secondaryIsActive) {
            return secondary.getWorldLinearAccelX();
        } else {
            return 0;
        }
    }

    public double getAccelerationY() {
        if (primaryIsActive) {
            return primary.getAccelerationY().getValueAsDouble();
        } else if (secondaryIsActive) {
            return secondary.getWorldLinearAccelY();
        } else {
            return 0;
        }
    }

    public boolean isPrimaryIsActive() {
        return primaryIsActive;
    }

    public boolean isSecondaryIsActive() {
        return secondaryIsActive;
    }
    
}
