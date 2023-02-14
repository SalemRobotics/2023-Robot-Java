package frc.robot;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.HashMap;
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
   * @return A hashmap containing each of the individual constants.
   */
    public static HashMap<String, Double> loadPIDConstants(String groupname, SparkMaxPIDController controller) {
        try (FileInputStream file = new FileInputStream("constants/PIDConstants.properties")) {
            RobotProperties properties = new RobotProperties();
            properties.load(file);
            
            HashMap<String, Double> output = new HashMap<String, Double>();

            // convert the property (group.property-name) to a double and assign it to a variable
            double p = Double.parseDouble(properties.getProperty(groupname, "Error-Multiplier(kP)"));
            double i = Double.parseDouble(properties.getProperty(groupname, "Sum-Error(kI)"));
            double d = Double.parseDouble(properties.getProperty(groupname, "Slope-Error(kD)"));
            double iz = Double.parseDouble(properties.getProperty(groupname, "Integral-Effective-Error-Range(kIz)"));
            double min = Double.parseDouble(properties.getProperty(groupname, "Min-Output(kMinOutput)"));
            double max = Double.parseDouble(properties.getProperty(groupname, "Max-Output(kMaxOutput)"));

            output.put("kP", p);
            output.put("kI", i);
            output.put("kD", d);
            output.put("kIz", iz);
            output.put("kMinOutput", min);
            output.put("kMaxOutput", max);

            // used by feedforward only
            double kS = Double.parseDouble(properties.getProperty(groupname, "Static-Gain(kS)"));
            double kG = Double.parseDouble(properties.getProperty(groupname, "Gravity-Gain(kG)"));
            double kV = Double.parseDouble(properties.getProperty(groupname, "Velocity-Gain(kV)"));
            double kA = Double.parseDouble(properties.getProperty(groupname, "Acceleration-Gain(kA)"));

            output.put("kS", kS);
            output.put("kG", kG);
            output.put("kV", kV);
            output.put("kA", kA);

            /* 
             * Will only be used by controllers using SmartMotion as their control type
             * otherwise these should be set to 0.0 for controllers using other control types. 
             */
            double minV = Double.parseDouble(properties.getProperty(groupname, "Min-Velocity(kMinV)"));
            double maxV = Double.parseDouble(properties.getProperty(groupname, "Max-Velocity(kMaxV)"));
            double maxA = Double.parseDouble(properties.getProperty(groupname, "Max-Acceleration(kMaxA)"));
            double allE = Double.parseDouble(properties.getProperty(groupname, "Allowed-Closed-Loop-Error(kAllowedErr)"));

            output.put("kMinV", minV);
            output.put("kMaxV", maxV);
            output.put("kMaxA", maxA);
            output.put("kAllowerErr", allE);

            // assign the property variable to the SparkMaxPIDController object.
            controller.setP(p);
            controller.setI(i);
            controller.setD(d);
            controller.setIZone(iz);
            controller.setOutputRange(min, max);
            controller.setSmartMotionMinOutputVelocity(minV, 0);
            controller.setSmartMotionMaxVelocity(maxV, 0);
            controller.setSmartMotionMaxAccel(maxA, 0);
            controller.setSmartMotionAllowedClosedLoopError(allE, 0);

            return output;
        } catch (IOException e) {
            DriverStation.reportError("File not found or corrupted.", e.getStackTrace());
            return null;
        }
    }
}
