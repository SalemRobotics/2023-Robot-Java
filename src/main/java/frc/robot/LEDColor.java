package frc.robot;

import edu.wpi.first.wpilibj.util.Color;

/**
 * Utility class for colors based on WPILib {@link Color}
 */
public class LEDColor extends Color {

    public double red, green, blue;
    public double hue, saturation, value;

    /**
     * Constructs an LEDColor.
     * @param red 0-255
     * @param green 0-255
     * @param blue 0-255
     */
    public LEDColor(double red, double green, double blue) {
        super(red, green, blue);
        this.red=red;
        this.green=green;
        this.blue=blue;
        calcHSV();
    }
    
    /**
     * Constructs an LEDColor.
     * @param red 0-255
     * @param green 0-255
     * @param blue 0-255
     */
    public LEDColor(int red, int green, int blue) {
        super(red, green, blue);
        this.red=red;
        this.green=green;
        this.blue=blue;
        calcHSV();
    }

    public void setRGB(double r, double g, double b) {
        red=r;
        green=g;
        blue=b;
        calcHSV();
    }

    public void setRGB(int r, int g, int b) {
        red=r;
        green=g;
        blue=b;
        calcHSV();
    }

    public void setHSV(double h, double s, double v) {
        hue=h;
        saturation=s;
        value=v;
        calcRGB();
    }

    /**
     * Linearly interpolates between two colors over time.
     * @param a First {@link Color}
     * @param b Second {@link Color}
     * @param t Time
     * @return The product color.
     */
    public static Color lerpRGB(Color a, Color b, double t) {
        return new Color(
            a.red + (b.red - a.red) * t,
            a.green + (b.green - a.green) * t,
            a.blue + (b.blue - a.blue) * t
        );
    }

    /**
     * Linearly interpolates between two colors over time.
     * @param a First {@link LEDColor}
     * @param b Second {@link LEDColor}
     * @param t Time
     * @return The product color.
     */
    public static LEDColor lerpHSV(LEDColor a, LEDColor b, double t) {
        double h=0; 
        double d = b.hue - a.hue;

        if (a.hue > b.hue) {
            double temp = b.hue;
            b.hue = a.hue;
            a.hue = temp;

            d = -d;
            t = 1 - t;
        }
        if (d > 0.5) {
            a.hue = a.hue + 1;
            h = (a.hue + t * (b.hue - a.hue)) % 1;
        }
        if (d <= 0.5) {
            h = a.hue + t * d;
        }

        LEDColor out = new LEDColor(0, 0, 0);
        out.setHSV(
            h, 
            a.saturation + t * (b.saturation - a.saturation), 
            a.value + t * (b.value - a.value)
        );
        return out;
    }
    
    /**
     * Calculates HSV from RGB
     */
    void calcHSV() {
        double cMax = Math.max(red, Math.max(green, blue));
        double cMin = Math.min(red, Math.min(green, blue));

        value = cMax/255;
        saturation = cMax > 0 ? 1 - cMin/cMax : 0;

        double a = red - (0.5 * green) - (0.5 * blue);
        double h = (red*red) + (green*green) + (blue*blue) - (red*green) - (red*blue) - (green*blue);
        double angle = Math.acos(a/h);
        if (green >= blue) {
            hue = Math.toDegrees(angle);
        }
        else if (blue > green) {
            hue = 360 - Math.toDegrees(angle);
        }
    }

    /**
     * Calculates RGB from HSV
     */
    void calcRGB() {
        double cMax = 255 * value;
        double cMin = cMax * (1 - saturation);
        double z = (cMax - cMin) * (1 - Math.abs((hue/60)%2 - 1));

        if (hue < 60) {
            red = cMax;
            green = z + cMin;
            blue = cMin;
        }
        else if (60 <= hue && hue < 120) {
            red = z + cMin;
            green = cMax;
            blue = cMin;
        }
        else if (120 <= hue && hue < 180) {
            red = cMin;
            green = cMax;
            blue = z + cMin;
        }
        else if (180 <= hue && hue < 240) {
            red = cMin;
            green = z + cMin;
            blue = cMax;
        }
        else if (240 <= hue && hue < 300) {
            red = z + cMin;
            green = cMin;
            blue = cMax;
        }
        else if (300 <= hue && hue < 360) {
            red = cMax;
            green = cMin;
            blue = z + cMin;
        }
    }
}
