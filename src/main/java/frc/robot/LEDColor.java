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

    public void setHSV(int h, int s, int v) {
        hue=h;
        saturation=s;
        value=v;
        calcRGB();
    }

    /**
     * Linearly interpolates between two colors over time.
     * @param a First {@link Color}
     * @param b Second {@link Color}
     * @param t Time, in seconds
     * @return The calculated color at t seconds
     */
    public static LEDColor lerpRGB(Color a, Color b, double t) {
        return new LEDColor(
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
        double rprime = red / 255;
        double gprime = green / 255;
        double bprime = blue / 255;

        double cMax = Math.max(rprime, Math.max(gprime, bprime));
        double cMin = Math.min(rprime, Math.min(gprime, bprime));

        double delta = cMax - cMin;

        value = cMax * 255;

        saturation = cMax == 0 ? 0 : (delta/cMax) * 255;

        if (delta == 0) 
            hue = 0;
        else if (cMax == rprime)
            hue = 30 * (((gprime - bprime)/delta) % 6);
        else if (cMax == gprime)
            hue = 30 * (((bprime - rprime)/delta) + 2);
        else if (cMax == bprime)
            hue = 30 * (((rprime - gprime)/delta) + 4);
    }

    /**
     * Calculates RGB from HSV
     */
    void calcRGB() {
        double cMax = value;
        double cMin = cMax * (1 - (saturation / 255));
        double z = (cMax - cMin) * (1 - Math.abs((hue / 30) % 2 - 1));

        if (hue < 30) {
            red = cMax;
            green = z + cMin;
            blue = cMin;
        }
        else if (30 <= hue && hue < 60) {
            red = z + cMin;
            green = cMax;
            blue = cMin;
        }
        else if (60 <= hue && hue < 90) {
            red = cMin;
            green = cMax;
            blue = z + cMin;
        }
        else if (90 <= hue && hue < 120) {
            red = cMin;
            green = z + cMin;
            blue = cMax;
        }
        else if (120 <= hue && hue < 150) {
            red = z + cMin;
            green = cMin;
            blue = cMax;
        }
        else if (150 <= hue && hue < 180) {
            red = cMax;
            green = cMin;
            blue = z + cMin;
        }
    }
}
