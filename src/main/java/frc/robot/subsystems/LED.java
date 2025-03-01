package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {

    private static final int LED_ADDRESSABLE_UNITS = 144;

    private AddressableLED addressableLED;
    private AddressableLEDBuffer buffer;
    private boolean ledRunning;

    public LED() {
        addressableLED = new AddressableLED(Constants.PwmPort.LED_PORT);
        buffer = new AddressableLEDBuffer(LED_ADDRESSABLE_UNITS);
        addressableLED.setLength(buffer.getLength());
    }

    public Command setLedColorWhileHeld(Color color) {
        LEDPattern redPattern = LEDPattern.solid(color);
        redPattern.applyTo(buffer);
        addressableLED.setData(buffer);

        return startEnd(() -> changeColor(color), () -> stopLEDs());
    }

    private void stopLEDs() {
        if (ledRunning) {
            addressableLED.stop();
            ledRunning = false;
        }
    }

    private void changeColor(Color color) {
        LEDPattern redPattern = LEDPattern.solid(Color.kRed);
        redPattern.applyTo(buffer);
        addressableLED.setData(buffer);

        if (!ledRunning) {
            addressableLED.start();
            ledRunning = true;
        }
    }
}
