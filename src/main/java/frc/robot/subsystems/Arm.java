package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import org.opencv.core.Point;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotProperties;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ArmPresets;

/**
 * The {@link Arm} subsystem controls a multistage telescoping arm utlizing 2 PID control systems
 * to control the position of the arm using (x,y) coordinates. <p>
 * Uses 3 NEO motors, 1 absolute encoder and 1 quadrature encoder.
 */
public class Arm extends SubsystemBase {
    public boolean isConeMode;

    CANSparkMax pivotMotor1 = new CANSparkMax(ArmConstants.kPivotPort1, MotorType.kBrushless);
    CANSparkMax pivotMotor2 = new CANSparkMax(ArmConstants.kPivotPort2, MotorType.kBrushless);

    CANSparkMax extensionMotor = new CANSparkMax(ArmConstants.kExtensionPort, MotorType.kBrushless);

    RelativeEncoder pivotEncoder = pivotMotor1.getEncoder();
    RelativeEncoder extensionEncoder = extensionMotor.getEncoder();

    DigitalInput pivotSwitchMin = new DigitalInput(ArmConstants.kPivotSwitchMinChannel);
    DigitalInput pivotSwitchMax = new DigitalInput(ArmConstants.kPivotSwitchMaxChannel);
    DigitalInput encoderSwitchMin = new DigitalInput(ArmConstants.kEncoderSwitchMinChannel);
    DigitalInput encoderSwitchMax = new DigitalInput(ArmConstants.kEncoderSwitchMaxChannel);

    SparkMaxPIDController pivotController = pivotMotor1.getPIDController();
    SparkMaxPIDController extensionController = extensionMotor.getPIDController();

    // TODO: multiply kG by cosine of the arm's angle relative to being vertical
    ElevatorFeedforward extenstionFeedforward; 

    HashMap<String, Double> extensionMap;

    TrapezoidProfile.Constraints extensionConstraints = new TrapezoidProfile.Constraints(0, 0);
    TrapezoidProfile.State extensionGoal = new TrapezoidProfile.State();
    TrapezoidProfile.State extensionCurrPoint = new TrapezoidProfile.State();
    TrapezoidProfile extensionProfile;
    
    /**
     * Constructs an Arm object that specifies the behavior of the PID controllers and encoders.
     */
    public Arm() {
        pivotMotor1.setIdleMode(IdleMode.kBrake);
        pivotMotor2.setIdleMode(IdleMode.kBrake);
        pivotMotor2.follow(pivotMotor1);

        extensionMotor.setIdleMode(IdleMode.kBrake);

        pivotEncoder.setPosition(0);

        // pivotEncoder.setDistancePerPulse(ArmConstants.kPivotEncoderDistance);
        // extensionEncoder.setPositionConversionFactor(ArmConstants.kExtensionEncoderDistance);

        extensionMap = RobotProperties.loadPIDConstants("ExtensionPID", extensionController);
        extenstionFeedforward = new ElevatorFeedforward(
            extensionMap.get("kS"),
            extensionMap.get("kG"),
            extensionMap.get("kV"),
            extensionMap.get("kA")
        );
    }
    
    /**
     * Moves the position of the {@link Arm} subsystem to the desired preset position.
     * @param preset A {@link Point} object representing a point in cartesian coordinate space to move to.
     */
    public CommandBase setTargetPoint(ArmPresets preset) {
        return run(
            () -> {
                setTargetPoint();
            }
        );
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
                setArmSpeeds(extension.getAsDouble(), rotation.getAsDouble());
                // limitResetEncoders();
            }
        );
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
     * Manually set the speeds for the extension and pivot of the {@link Arm} subsystem
     * @param ext the speed to extend at
     * @param rot the speed to pivot at
     */
    void setArmSpeeds(double ext, double rot) {
        if (isAtLimit(pivotEncoder.getPosition(), ArmConstants.kMinPivotAngle, ArmConstants.kMaxPivotAngle, rot))
            pivotMotor1.set(0.0);
        else 
            pivotMotor1.set(rot);

        // if (getCurrentPoint().y >= ArmConstants.kMaxHeight)
        //     extensionMotor.set(-1.0);
        if (isAtLimit(extensionEncoder.getPosition(), 0.0, ArmConstants.kArmMaxExtensionLength, ext))
            extensionMotor.set(0.0);
        else
            extensionMotor.set(ext);
    }

    /**
     * Sets the target position for the end effector to travel towards.
     * @param point A {@link Point} object representing a point in cartesian coordinate space to move to.
     */
    void setTargetPoint() {
        double angle = pivotEncoder.getPosition(); // convert to radians
        
        extensionController.setReference(extensionCurrPoint.position, ControlType.kPosition, 0, 
            (extenstionFeedforward.calculate(extensionEncoder.getVelocity()) * Math.sin(angle))
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot", pivotEncoder.getPosition());
        
        SmartDashboard.putNumber("Pivot Output", pivotMotor1.getAppliedOutput());
    }
}
