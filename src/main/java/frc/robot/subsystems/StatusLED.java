package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LEDColor;
import frc.robot.constants.LEDConstants;

public class StatusLED extends SubsystemBase {
    AddressableLED led = new AddressableLED(LEDConstants.ledPort);
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.ledLength);

    boolean isOn = false;
    
    public StatusLED() {
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    /**
     * Sets a solid color for the whole LED strip. 
     * @param color an RGB color object
     */
    void setStripColor(Color color) {
        for (int i=0; i < ledBuffer.getLength(); i++) {
            setRGB(i, color);
        }
        led.setData(ledBuffer);
    }

    CommandBase startColor(Color color) {
        return run(
            () -> {
                setStripColor(color);
            }
        );
    }

    CommandBase blinkStripColor(Color color, double interval) {
        Timer timer = new Timer();
        return new FunctionalCommand(
            () -> { // init
                setStripColor(color);
                timer.start();
                isOn = true;
            }, 
            () -> { // exec
                if (timer.get() % interval == 0) {
                    if (isOn) {
                        setStripColor(Color.kBlack);
                        isOn = false;
                    } else {
                        setStripColor(color);
                        isOn = true;
                    }
                }
            }, 
            null, 
            null, 
            this
        );
    }

    CommandBase hueShiftColor(Color color, double interval) {
        Timer timer = new Timer();
        Color hueColor = new Color(color.red, color.green, color.blue);
        return new FunctionalCommand(
            () -> { // init
                setStripColor(hueColor);
                timer.start();
            }, 
            () -> { // exec
                setStripColor(hueColor);
                if (timer.get() % interval == 0 && color != Color.kBlack) {
                    
                }
            }, 
            null, 
            null, 
            this
        );
    }

    /**
     * Overloaded setRGB function to utilize color objects.
     * @param index the index to write
     * @param color 12 bit RGB color object
     */
    void setRGB(int index, Color color) {
        ledBuffer.setRGB(index, (int)color.red, (int)color.green, (int)color.blue);
    }
}
