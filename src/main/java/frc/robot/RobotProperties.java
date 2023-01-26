package frc.robot;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Class to load and read properties relating to systems of the robot (particularly PID).
 */
public class RobotProperties extends Properties {
    /**
     * Load properties in the format "group.property-name".
     * @param group The group name.
     * @param key The property name.
     */
    public String getProperty(String group, String key) {
        return getProperty(group + '.' + key);
    }

    /**
   * Method to read constants from a .properties file and assign them to a SparkMaxPIDController object. 
   * @param groupname The name of the PID constant group (e.g PivotPID for the Pivoting closed loop).
   * @param controller A SparkMaxPIDController Object.
   */
    public static void loadPIDConstants(String groupname, SparkMaxPIDController controller) {
        try (FileInputStream file = new FileInputStream("constants/PIDConstants.properties")) {
            RobotProperties properties = new RobotProperties();
            properties.load(file);

            // convert the property (group.property-name) to a double and assign it to a variable
            double p = Double.parseDouble(properties.getProperty(groupname, "Error-Multiplier(P)"));
            double i = Double.parseDouble(properties.getProperty(groupname, "Sum-Error(I)"));
            double d = Double.parseDouble(properties.getProperty(groupname, "Slope-Error(D)"));
            double iz = Double.parseDouble(properties.getProperty(groupname, "Integral-Effective-Error-Range(IZone)"));
            double ff = Double.parseDouble(properties.getProperty(groupname, "Control-Loop-Gain(Feed-Forward)"));
            double min = Double.parseDouble(properties.getProperty(groupname, "Min-Output"));
            double max = Double.parseDouble(properties.getProperty(groupname, "Max-Output"));

            /* 
             * Will only be used by controllers using SmartMotion as their control type
             * otherwise these should be set to 0.0 for controllers using other control types. 
             */
            double minV = Double.parseDouble(properties.getProperty(groupname, "Min-Velocity"));
            double maxV = Double.parseDouble(properties.getProperty(groupname, "Max-Velocity"));
            double maxA = Double.parseDouble(properties.getProperty(groupname, "Max-Acceleration"));
            double allE = Double.parseDouble(properties.getProperty(groupname, "Allowed-Closed-Loop-Error"));

            // assign the property variable to the SparkMaxPIDController object.
            controller.setP(p);
            controller.setI(i);
            controller.setD(d);
            controller.setIZone(iz);
            controller.setFF(ff);
            controller.setOutputRange(min, max);
            controller.setSmartMotionMinOutputVelocity(minV, 0);
            controller.setSmartMotionMaxVelocity(maxV, 0);
            controller.setSmartMotionMaxAccel(maxA, 0);
            controller.setSmartMotionAllowedClosedLoopError(allE, 0);
        } catch (IOException e) {
            DriverStation.reportError("File not found or corrupted.", e.getStackTrace());
        }
    }
}
