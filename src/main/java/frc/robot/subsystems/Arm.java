package frc.robot.subsystems;

import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.opencv.core.Point;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ArmPresets;

/**
 * The {@link Arm} subsystem controls a multistage telescoping arm utlizing 2 PID control systems
 * to control the position of the arm using (x,y) coordinates. <p>
 * Uses 3 NEO motors, 1 absolute encoder and 1 quadrature encoder.
 */
public class Arm extends SubsystemBase {
    public static boolean isConeMode = false;

    CANSparkMax pivotMotor1 = new CANSparkMax(ArmConstants.kPivotPort1, MotorType.kBrushless);
    CANSparkMax pivotMotor2 = new CANSparkMax(ArmConstants.kPivotPort2, MotorType.kBrushless);

    CANSparkMax extensionMotor = new CANSparkMax(ArmConstants.kExtensionPort, MotorType.kBrushless);

    RelativeEncoder pivotEncoder = pivotMotor1.getEncoder();
    RelativeEncoder extensionEncoder = extensionMotor.getEncoder();

    DigitalInput pivotSwitchMin = new DigitalInput(ArmConstants.kPivotSwitchMinChannel);
    DigitalInput pivotSwitchMax = new DigitalInput(ArmConstants.kPivotSwitchMaxChannel);
    DigitalInput extensionSwitchMin = new DigitalInput(ArmConstants.kExtensionSwitchMinChannel);
    DigitalInput extensionSwitchMax = new DigitalInput(ArmConstants.kExtensionSwitchMaxChannel);

    
    // csv data
    List<String[]> datalines = new ArrayList<>();
    Path newfilePath = Filesystem.getOperatingDirectory().toPath().resolve("tuning.csv");

    /**
     * Constructs an Arm object that specifies the behavior of the PID controllers and encoders.
     */
    public Arm() {
        pivotMotor2.follow(pivotMotor1);
        datalines.add(new String[] {"Pivot:", "Pivot Output:", "Extension:", "Extension Output:"});
        try {
            Files.createFile(newfilePath);
            writeCSV();
            System.out.println("File created successfully");
        } catch (IOException e) {
            System.out.println("File already exists or path not found");
            e.printStackTrace();
        }
    }
    
    /**
     * Moves the position of the {@link Arm} subsystem using manual operator controls. <p>
     * Gives the ability to control both rotation and extension of the arm.
     * @param extension the speed to extend at
     * @param rotation the speed to pivot at
     */
    public CommandBase setArmSpeeds(DoubleSupplier extension, DoubleSupplier rotation) {
        return run(
            () -> {
                double rot = rotation.getAsDouble();
                double ext = extension.getAsDouble();

                if (isAtLimit(pivotEncoder.getPosition(), ArmConstants.kMinPivotAngle, ArmConstants.kMaxPivotAngle, rot))
                    pivotMotor1.set(0.0);
                else 
                    pivotMotor1.set(
                        lerpRequiredOutput(pivotEncoder.getPosition(), extensionEncoder.getPosition()) + 0.25 * rot
                    );
        
                double extPosInches = extensionEncoder.getPosition() * ArmConstants.kExtensionDistanceFactor;
                // if (getCurrentPoint().y >= ArmConstants.kMaxHeight)
                //     extensionMotor.set(-1.0); 
                if (isAtLimit(extPosInches, 0.0, ArmConstants.kArmMaxExtensionLength, ext))
                    extensionMotor.set(0.0);
                else
                    extensionMotor.set(ext/2);
            }
        );
    }

    /**
     * Moves the position of the {@link Arm} subsystem to the desired preset position.
     * @param preset A {@link Point} object representing a point in cartesian coordinate space to move to.
     */
    public CommandBase setTargetPoint(ArmPresets cubePreset, ArmPresets conePreset) {
        return run(
            () -> {
                double pivotPreset = isConeMode ? conePreset.value.x : cubePreset.value.x;
                double pivotError = pivotPreset - pivotEncoder.getPosition();
                double pivotProportional = ArmConstants.kPPivot * pivotError;
                pivotProportional = checkSpeedLimit(pivotProportional, ArmConstants.kPivotMaxSpeed);
                pivotMotor1.set(
                    lerpRequiredOutput(pivotEncoder.getPosition(), extensionEncoder.getPosition()) + pivotProportional
                );

                double extPreset = isConeMode ? conePreset.value.y : cubePreset.value.y;
                double extError = extPreset - extensionEncoder.getPosition();
                double extensionProportional = ArmConstants.kPExtension * extError;
                extensionProportional = checkSpeedLimit(extensionProportional, ArmConstants.kExtensionMaxSpeed);
                extensionMotor.set(extensionProportional);
                // extensionMotor.set(extensionProportional * mapPivotAngle(pivotEncoder.getPosition()));
            }
        );
    }

    /**
     * Clamps the desired error between a max/min speed
     * @param error Desired error
     * @param limit Speed limit
     * @return The clamped error
     */
    double checkSpeedLimit(double error, double limit) {
        double outError=error;
        if(outError > limit) {
            outError = limit;
        }
        if (outError < -limit) {
            outError = -limit;
        }
        return outError;
    }

    /**
     * Checks if the specified encoder position is within the defined limits
     * @param encoderPosition the encoder position to check
     * @param min minimum constraint
     * @param max maximum constraint
     * @param requestedSpeed requested speed that the motor will attempt to run at
     * @return true if limit is reached
     */
    boolean isAtLimit(double encoderPosition, double min, double max, double requestedSpeed) {
        // return (encoderPosition >= max && requestedSpeed > 0) || (encoderPosition <= min && requestedSpeed < 0);
        return false;
    }

    /**
     * Resets either the pivot or extension encoders to 0 if their respective limit switches are pressed
     */
    void checkResetEncoders() {
        if (pivotSwitchMin.get()) pivotEncoder.setPosition(0);
        else if (pivotSwitchMax.get()) pivotEncoder.setPosition(0);

        if (extensionSwitchMin.get()) extensionEncoder.setPosition(0);
        else if (extensionSwitchMax.get()) extensionEncoder.setPosition(0);
    }

    /**
     * Maps the pivot's encoder to degrees. 
     * @param encoderPos Current encoder position
     * @return Current pivot angle, in degrees
     */
    double mapPivotAngle(double encoderPos) {
        
        return 0;
    }

    /**
     * Creates a feed forward value based on the encoder position. Data collected at minimum extension.
     * @param encoderPos Encoder position
     * @return Feed forward
     */
    double mapEncoderOutputIn(double encoderPos) {
        double deg1 = 6.37E-3 * encoderPos;
        double deg2 = -2.8E-3 * Math.pow(encoderPos, 2);
        double deg3 = -4.27E-4 * Math.pow(encoderPos, 3);
        double deg4 = -1.14E-5 * Math.pow(encoderPos, 4);
        return 0.0113 + deg1 + deg2 + deg3 + deg4;
    }

    /**
     * Creates a feed forward value based on the encoder position. Data collected at minimum extension.
     * @param encoderPos Encoder position
     * @return Feed forward
     */
    double mapEncoderOutputOut(double encoderPos) {
        // 4th degree polynomial to calculate the feed forward.
        // Essentially equal to the 'kS' value in a WPILib feed forward.
        double deg1 = 0.0166 * encoderPos;
        double deg2 = -0.0103 * Math.pow(encoderPos, 2);
        double deg3 = -1.94E-3 * Math.pow(encoderPos, 3);
        double deg4 = -7.97E-5 * Math.pow(encoderPos, 4);
        return 4.67E-3 + deg1 + deg2 + deg3 + deg4;
    }

    /**
     * Interpolates between the minimum and maximum extension feed forwards to produce a feed forward at desired extension
     * @param pivotPos Pivot encoder position
     * @param extensionPos Extension encoder position
     * @return Interpolated feed forward
     */
    double lerpRequiredOutput(double pivotPos, double extensionPos) {
        if(pivotPos > 0){
            // Minimum allowed pivot position
            return 0.01;
        }
        double a = mapEncoderOutputIn(pivotPos) * (0.0260078 * extensionPos + 1.0130039);
        double b = mapEncoderOutputOut(pivotPos) * (-0.0260078 * extensionPos - 0.0130039);

        return a + b;
    }

    public CommandBase snapshotEncoderPosition() {
        return runOnce(
            () -> {
                String pivotPos = Double.toString(pivotEncoder.getPosition());
                String pivotOut = Double.toString(pivotMotor1.getAppliedOutput());
                String extensionPos = Double.toString(extensionEncoder.getPosition());
                String extensionOut = Double.toString(extensionMotor.getAppliedOutput());
                datalines.add(new String[] {pivotPos, pivotOut, extensionPos, extensionOut});
                try {
                    writeCSV();
                    System.out.println("File successfully written");
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        );
    }

    void writeCSV() throws IOException {
        try (PrintWriter pw = new PrintWriter(newfilePath.toString())) {
            datalines.stream()
            .map(this::toCSV)
            .forEach(pw::println);
        }
    }

    String toCSV(String[] data) {
        return Stream.of(data)
        .map(this::escapeSpecialChars)
        .collect(Collectors.joining(","));
    }

    String escapeSpecialChars(String data) {
        String escapedData = data.replaceAll("\\R", " ");
        if (data.contains(",") || data.contains("\"") || data.contains("'")) {
            data = data.replace("\"", "\"\"");
            escapedData = "\"" + data + "\"";
        }
        return escapedData;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Encoder", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Pivot Output", pivotMotor1.getAppliedOutput());
        
        SmartDashboard.putNumber("Extension Encoder", extensionEncoder.getPosition());
        SmartDashboard.putNumber("Extension Output", extensionMotor.getAppliedOutput());
        SmartDashboard.putNumber("Extension Current", extensionMotor.getOutputCurrent());

        SmartDashboard.putBoolean("Cone Mode", Arm.isConeMode);
    }
}
