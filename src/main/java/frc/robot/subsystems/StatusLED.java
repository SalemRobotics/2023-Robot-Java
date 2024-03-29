package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LEDColor;
import frc.robot.constants.LEDConstants;

/**
 * Subsystem to control Addressable LED strips using Commands.
 */
public class StatusLED extends SubsystemBase {
    final AddressableLED led = new AddressableLED(LEDConstants.ledPort);
    final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.ledLength);

    final Timer timer = new Timer();

    boolean isOn = false;
    
    public StatusLED() {
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    /**
     * Display color to indicate which gamepiece is going to be picked up.
     * @return {@linkplain Command}
     */
    public CommandBase gamepieceSolidColor() {
        return run(
            () -> {
                if (Arm.isConeMode) {
                    setStripColorHSV(new LEDColor(255, 255, 0));
                }
                else {
                    setStripColorHSV(new LEDColor(100, 0, 200));
                }
            }
        );
    }

    /**
     * Blink green when the intake is at current limit.
     * @return {@linkplain Command}
     */
    public Command overCurrentBlink() {
        return blinkStripColor(new LEDColor(0, 255, 0), new LEDColor(0, 0, 0), 0.2);
    }

    public Command testLerpColor() {
        return interpolateStripColor(new LEDColor(255, 0, 0), new LEDColor(0, 0, 255));
    }

    /**
     * Slow breathes alliance color when disabled.
     * @return {@linkplain Command}
     */
    public Command breathTeamColor() {
        Alliance all = DriverStation.getAlliance();
        if (all == Alliance.Red) {
            return slowBlinkStripColor(new LEDColor(255, 0, 0), 1).ignoringDisable(true);
        }
        return slowBlinkStripColor(new LEDColor(0, 0, 255), 1).ignoringDisable(true);
    }

    /**
     * Displays alliance color when enabled.
     * @return {@linkplain Command}
     */
    public CommandBase solidTeamColor() {
        return run(
            () -> {
                Alliance all = DriverStation.getAlliance();
                if (all == Alliance.Red) {
                    setStripColorHSV(new LEDColor(255, 0, 0));
                }
                else {
                    setStripColorHSV(new LEDColor(0, 0, 255));
                }
            }
        );
    }

    /**
     * Blinks the LED strip between 2 colors.
     * @param color1 first {@link Color}
     * @param color2 second {@link Color}
     * @param interval the interval between blinks
     * @return a {@link FunctionalCommand}
     */
    CommandBase blinkStripColor(LEDColor color1, LEDColor color2, double interval) {
        return new FunctionalCommand(
            () -> { // init
                setStripColorHSV(color1);
                timer.reset();
                timer.start();
                isOn = false;
            }, 
            () -> { // exec
                isOn = timer.hasElapsed(interval);
                if (isOn && timer.hasElapsed(interval*2)) {
                    timer.restart();
                }

                if (isOn)
                    setStripColorHSV(color2);
                else 
                    setStripColorHSV(color1);
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
    CommandBase interpolateStripColor(LEDColor a, LEDColor b) {
        return new FunctionalCommand(
            () -> { // init
                setStripColorHSV(a);
                timer.reset();
                timer.start();
                isOn = true;
            }, 
            () -> { // exec
                LEDColor lerpColor = a;
                if (isOn) {
                    lerpColor = LEDColor.lerpRGB(a, b, timer.get());
                    if (!lerpColor.equals(b)) {
                        isOn = false;
                        timer.restart();
                    }
                } else {
                    lerpColor = LEDColor.lerpRGB(b, a, timer.get());
                    if (lerpColor.equals(a)) {
                        isOn = true;
                        timer.restart();
                    }
                }
                setStripColorHSV(lerpColor);
                SmartDashboard.putBoolean("isOn", isOn);
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
    CommandBase slowBlinkStripColor(LEDColor color, double speed) {
        return new FunctionalCommand(
            () -> { // init
                setStripColorHSV(color);
                isOn = true;
            },
            () -> { // exec
                LEDColor hsvColor = color;
                if (isOn){
                    hsvColor.value = hsvColor.value > 0 ? hsvColor.value -= speed*2.55 : 0;
                    hsvColor.setHSV(hsvColor.hue, hsvColor.saturation, hsvColor.value);
                    isOn = !(hsvColor.value == 0);
                }
                else {
                    hsvColor.value = hsvColor.value < 255 ? hsvColor.value += speed*2.55 : 255;
                    hsvColor.setHSV(hsvColor.hue, hsvColor.saturation, hsvColor.value);
                    isOn = hsvColor.value == 255;
                }
                setStripColorHSV(hsvColor);
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
            setHSV(i, color);
        }
        led.setData(ledBuffer);
    }
    
    void setStripColorHSV(int h, int s, int v) {
        for (int i=0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, h, s, v);
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

    void setHSV(int index, LEDColor color) {
        ledBuffer.setHSV(index, (int)color.hue, (int)color.saturation, (int)color.value);
    }
}
