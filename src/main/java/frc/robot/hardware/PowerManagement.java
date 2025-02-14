package frc.robot.hardware;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class PowerManagement {

    private static final int NUM_PDH_CHANNELS = 24;
    private final PowerDistribution powerDistributionHub;
    private final Alert brownoutAlert = new Alert("PDH Brownout detected!", AlertType.kWarning);

    public PowerManagement() {
        powerDistributionHub = new PowerDistribution(Constants.CanID.POWER_DISTRIBUTION, ModuleType.kRev);
    }

    public void updateTelemetry() {
        SmartDashboard.putNumber("Input Voltage", powerDistributionHub.getVoltage());
        SmartDashboard.putNumber("Total Current", powerDistributionHub.getTotalCurrent());

        if (powerDistributionHub.getFaults().Brownout) {
            brownoutAlert.set(true);
        }

        //TODO: Create a Map of PDH channel IDs to CAN devices

        for (int channel = 0; channel < NUM_PDH_CHANNELS; channel++) {
            if (powerDistributionHub.getFaults().getBreakerFault(channel)) {
                new Alert("PDH Breaker fault detected on channel " + channel + " - current = " + powerDistributionHub.getCurrent(channel), AlertType.kWarning).set(true);
            }
        }
    }
}
