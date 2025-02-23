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
        channelsInUse.add(0);
        channelsInUse.add(1);
        channelsInUse.add(2);
        channelsInUse.add(3);
        channelsInUse.add(4);
        channelsInUse.add(8);
        channelsInUse.add(9);
        channelsInUse.add(10);
        channelsInUse.add(13);
        channelsInUse.add(15);
        channelsInUse.add(16);
        channelsInUse.add(17);
        channelsInUse.add(18);
        channelsInUse.add(19);
        channelsInUse.add(20);
        channelsInUse.add(22);
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
