package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.LEDConstants;

public class StatusLED extends SubsystemBase {
    AddressableLED led = new AddressableLED(LEDConstants.ledPort);
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.ledLength);
    
    public StatusLED() {
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    void setStripColor(Color color) {
        for (int i=0; i < ledBuffer.getLength(); i++) {
            setRGB(i, color);
        }
        led.setData(ledBuffer);
    }

    void blinkStripColor(Color color, double interval) {
        WaitCommand wait = new WaitCommand(interval);
        for (int i=0; i < ledBuffer.getLength(); i++) {
            setRGB(i, color);
            if (wait.isFinished()) {
                setRGB(i, null);
            }
        }
        led.setData(ledBuffer);
    }

    void setRGB(int index, Color color) {
        ledBuffer.setRGB(index, (int)color.red, (int)color.green, (int)color.blue);
    }
}
