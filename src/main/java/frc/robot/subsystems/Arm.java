package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import org.opencv.core.Point;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
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
    public boolean isConeMode = false;

    CANSparkMax pivotMotor1 = new CANSparkMax(ArmConstants.kPivotPort1, MotorType.kBrushless);
    CANSparkMax pivotMotor2 = new CANSparkMax(ArmConstants.kPivotPort2, MotorType.kBrushless);

    CANSparkMax extensionMotor = new CANSparkMax(ArmConstants.kExtensionPort, MotorType.kBrushless);

    RelativeEncoder pivotEncoder = pivotMotor1.getEncoder();
    RelativeEncoder extensionEncoder = extensionMotor.getEncoder();

    DigitalInput pivotSwitchMin = new DigitalInput(ArmConstants.kPivotSwitchMinChannel);
    DigitalInput pivotSwitchMax = new DigitalInput(ArmConstants.kPivotSwitchMaxChannel);
    DigitalInput encoderSwitchMin = new DigitalInput(ArmConstants.kExtensionSwitchMinChannel);
    DigitalInput encoderSwitchMax = new DigitalInput(ArmConstants.kExtensionSwitchMaxChannel);

    Constraints pivotConstraints = new Constraints(ArmConstants.kMaxPivotVelocity, ArmConstants.kMaxPivotAccel);
    State pivotGoal;
    State pivotCurrentState;

    /**
     * Constructs an Arm object that specifies the behavior of the PID controllers and encoders.
     */
    public Arm() {
        pivotMotor2.follow(pivotMotor1);
        extensionMotor.setSmartCurrentLimit(40);
        extensionMotor.burnFlash();
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
        
                // if (getCurrentPoint().y >= ArmConstants.kMaxHeight)
                //     extensionMotor.set(-1.0); 
                if (isAtLimit(extensionEncoder.getPosition(), 0.0, ArmConstants.kArmMaxExtensionLength, ext))
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
    public CommandBase setTargetPoint(ArmPresets preset) {
        return run(
            () -> {
                double pivotError = preset.value.x - pivotEncoder.getPosition();
                double pivotProportional = ArmConstants.kPPivot * pivotError;
                pivotProportional = checkSpeedLimit(pivotProportional, ArmConstants.kPivotMaxSpeed);

                pivotGoal = new State(pivotProportional, 0);
                var profile = new TrapezoidProfile(pivotConstraints, pivotGoal, pivotCurrentState);
                pivotCurrentState = profile.calculate(0.02);  

                pivotMotor1.set(
                    lerpRequiredOutput(pivotEncoder.getPosition(), extensionEncoder.getPosition()) + pivotCurrentState.position
                );
                     
                double extError = preset.value.y - extensionEncoder.getPosition();
                double extensionProportional = ArmConstants.kPExtension * extError;
                extensionProportional = checkSpeedLimit(extensionProportional, ArmConstants.kExtensionMaxSpeed);
                extensionMotor.set(extensionProportional);
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
    void limitResetEncoders() {
        if (pivotSwitchMin.get() || pivotSwitchMax.get()) {
            pivotEncoder.setPosition(0);
        }
        if (encoderSwitchMin.get() || encoderSwitchMax.get()) {
            extensionEncoder.setPosition(0);
        }
    }

    /**
     * Creates a feed forward value at the minimum extension
     * @param encoderPos Encoder position
     * @return Feed forward
     */
    double mapEncoderOutputIn(double encoderPos) {
        double deg1 = 0.00679 * encoderPos;
        double deg2 = -0.00549 * Math.pow(encoderPos, 2);
        double deg3 = -0.00103 * Math.pow(encoderPos, 3);
        double deg4 = -0.000043 * Math.pow(encoderPos, 4);
        return 0.000134 + deg1 + deg2 + deg3 + deg4;
    }

    /**
     * Creates a feed forward value at the maximum extension
     * @param encoderPos Encoder position
     * @return Feed forward
     */
    double mapEncoderOutputOut(double encoderPos) {
        // 4th degree polynomial to calculate the feed forward.
        // Essentially equal to the 'kS' value in a WPILib feed forward.
        double deg1 = 0.0115 * encoderPos;
        double deg2 = -0.0117 * Math.pow(encoderPos, 2);
        double deg3 = -0.00176 * Math.pow(encoderPos, 3);
        double deg4 = -0.0000565 * Math.pow(encoderPos, 4);
        return -0.00224 + deg1 + deg2 + deg3 + deg4;
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Encoder", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Pivot Output", pivotMotor1.getAppliedOutput());
        
        SmartDashboard.putNumber("Extension Encoder", extensionEncoder.getPosition());
        SmartDashboard.putNumber("Extension Output", extensionMotor.getAppliedOutput());
        SmartDashboard.putNumber("Extension Current", extensionMotor.getOutputCurrent());
    }
}
