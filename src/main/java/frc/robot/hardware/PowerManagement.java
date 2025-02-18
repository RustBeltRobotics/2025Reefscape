package frc.robot.hardware;

import java.util.HashSet;

import java.util.Set;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.AlertManager;

public class PowerManagement {

    private static final int NUM_PDH_CHANNELS = 24;
    private final PowerDistribution powerDistributionHub;
    private final Alert brownoutAlert = new Alert("PDH Brownout detected!", AlertType.kWarning);
    private final Set<Integer> channelsInUse;  //avoid alerting on channels we know are not in active use

    public PowerManagement() {
        powerDistributionHub = new PowerDistribution(Constants.CanID.POWER_DISTRIBUTION, ModuleType.kRev);
        channelsInUse = new HashSet<>();
        //TODO: Define which channels are in use
    }

    public void updateTelemetry() {
        SmartDashboard.putNumber("Input Voltage", powerDistributionHub.getVoltage());
        SmartDashboard.putNumber("Total Current", powerDistributionHub.getTotalCurrent());

        if (powerDistributionHub.getFaults().Brownout) {
            brownoutAlert.set(true);
        }

        //TODO: Create a Map of PDH channel IDs to CAN devices to make it clearer which device is causing the fault
        for (int channel = 0; channel < NUM_PDH_CHANNELS; channel++) {
            if (powerDistributionHub.getFaults().getBreakerFault(channel) && channelsInUse.contains(channel)) {
                AlertManager.addAlert("PDH-Channel-" + channel, "PDH Breaker fault detected on channel " + channel + " - current = " + powerDistributionHub.getCurrent(channel), AlertType.kWarning);
            } else {
                AlertManager.removeAlert("PDH-Channel-" + channel);
            }
        }
    }
}
