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

/**
 * Subsystem to control Addressable LED strips using Commands.
 */
public class StatusLED extends SubsystemBase {
    AddressableLED led = new AddressableLED(LEDConstants.ledPort);
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.ledLength);

    Timer timer = new Timer();

    boolean isOn = false;
    
    public StatusLED() {
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    /**
     * Blinks the LED strip between 2 colors.
     * @param color1 first {@link Color}
     * @param color2 second {@link Color}
     * @param interval the interval between blinks
     * @return a {@link FunctionalCommand}
     */
    CommandBase blinkStripColor(Color color1, Color color2, double interval) {
        return new FunctionalCommand(
            () -> { // init
                setStripColorRGB(color1);
                timer.reset();
                timer.start();
                isOn = true;
            }, 
            () -> { // exec
                if (timer.get() % interval == 0) {
                    if (isOn) {
                        setStripColorRGB(color2);
                        isOn = false;
                    } else {
                        setStripColorRGB(color1);
                        isOn = true;
                    }
                }
            }, 
            isFinished -> {
                timer.stop();
            }, 
            () -> { return false; },
            this
        );
    }

    /**
     * Linearly interpolates back and forth between two colors. 
     * @param a First {@link Color}
     * @param b Second {@link Color}
     * @return A {@link FunctionalCommand}
     */
    CommandBase interpolateStripColor(Color a, Color b) {
        return new FunctionalCommand(
            () -> { // init
                setStripColorRGB(b);
                timer.reset();
                timer.start();
                isOn = true;
            }, 
            () -> { // exec
                Color lerpColor;
                if (isOn) {
                    lerpColor = LEDColor.lerpRGB(a, b, timer.get());
                    setStripColorRGB(lerpColor);
                    isOn = !lerpColor.equals(b);
                } else {
                    lerpColor = LEDColor.lerpRGB(b, a, timer.get());
                    setStripColorRGB(lerpColor);
                    isOn = lerpColor.equals(a);
                }
            }, 
            isFinished -> {
                timer.stop();
            }, 
            () -> { return false; }, 
            this
        );
    }

    /**
     * Shifts the value of an inputted {@link Color} down to black and back up again, making a slow blink.
     * @param color the initial color
     * @param speed the speed of which the color shifts.
     * @return a {@link FunctionalCommand}
     */
    CommandBase slowBlinkStripColor(Color color, double speed) {
        final LEDColor hsvColor = (LEDColor) color;
        return new FunctionalCommand(
            () -> { // init
                setStripColorRGB(color);
                isOn = true;
            },
            () -> { // exec
                if (isOn) {
                    hsvColor.value = hsvColor.value != 0 ? hsvColor.value -= speed : 0;
                    setStripColorHSV(hsvColor);
                    isOn = !hsvColor.equals(Color.kBlack);
                }
                else {
                    hsvColor.value = hsvColor.value != 1 ? hsvColor.value += speed : 1;
                    setStripColorHSV(hsvColor);
                    isOn = hsvColor.equals(color);
                }
            },
            isFinished -> {},
            () -> { return false; },
            this
        );
    }

    /**
     * Sets a solid color for the whole LED strip. 
     * @param color an RGB {@link Color}
     */
    void setStripColorRGB(Color color) {
        for (int i=0; i < ledBuffer.getLength(); i++) {
            setRGB(i, color);
        }
        led.setData(ledBuffer);
    }

    /**
     * Sets a solid color for the Whole LED strip.
     * @param color An HSV {@linkplain LEDColor}
     */
    void setStripColorHSV(LEDColor color) {
        for (int i=0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, (int)color.hue, (int)color.saturation, (int)color.value);
        }
        led.setData(ledBuffer);
    }

    /**
     * Overloaded setRGB function to utilize color objects.
     * @param index the index to write
     * @param color 12 bit RGB {@link Color}
     */
    void setRGB(int index, Color color) {
        ledBuffer.setRGB(index, (int)color.red, (int)color.green, (int)color.blue);
    }
}
