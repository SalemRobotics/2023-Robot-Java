package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import org.opencv.core.Point;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
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

    Encoder pivotEncoder = new Encoder(ArmConstants.kPivotEncoderSourceA, ArmConstants.kPivotEncoderSourceB);
    RelativeEncoder extensionEncoder = extensionMotor.getEncoder();

    DigitalInput pivotSwitchMin = new DigitalInput(ArmConstants.kPivotSwitchMinChannel);
    DigitalInput pivotSwitchMax = new DigitalInput(ArmConstants.kPivotSwitchMaxChannel);
    DigitalInput encoderSwitchMin = new DigitalInput(ArmConstants.kEncoderSwitchMinChannel);
    DigitalInput encoderSwitchMax = new DigitalInput(ArmConstants.kEncoderSwitchMaxChannel);

    SparkMaxPIDController pivotController;
    SparkMaxPIDController extensionController;

    ArmFeedforward pivotFeedforward;
        // TODO: multiply kG by cosine of the arm's angle relative to being vertical
    ElevatorFeedforward extenstionFeedforward; 

    HashMap<String, Double> pivotMap;
    HashMap<String, Double> extensionMap;

    TrapezoidProfile.Constraints pivotConstraints = new TrapezoidProfile.Constraints(0, 0);
    TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State();
    TrapezoidProfile.State pivotCurrPoint = new TrapezoidProfile.State();
    TrapezoidProfile pivotProfile;

    TrapezoidProfile.Constraints extensionConstraints = new TrapezoidProfile.Constraints(0, 0);
    TrapezoidProfile.State extensionGoal = new TrapezoidProfile.State();
    TrapezoidProfile.State extensionCurrPoint = new TrapezoidProfile.State();
    TrapezoidProfile extensionProfile;
    
    /**
     * Constructs an Arm object that specifies the behavior of the PID controllers and encoders.
     */
    public Arm() {
        pivotMotor2.follow(pivotMotor1);

        pivotEncoder.setDistancePerPulse(ArmConstants.kPivotEncoderDistance);
        extensionEncoder.setPositionConversionFactor(ArmConstants.kExtensionEncoderDistance);

        pivotMap = RobotProperties.loadPIDConstants("PivotPID", pivotController);
        pivotFeedforward = new ArmFeedforward(
            pivotMap.get("kS"),
            pivotMap.get("kG"),
            pivotMap.get("kV"),
            pivotMap.get("kA")
        );

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
                setTargetPoint(preset.value);
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
                setArmSpeeds(extension.getAsDouble()/2, rotation.getAsDouble()/2);
                limitResetEncoders();
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
        return (encoderPosition >= max && requestedSpeed > 0) || (encoderPosition <= min && requestedSpeed < 0);
    }

    /**
     * Resets either the pivot or extension encoders to 0 if their respective limit switches are pressed
     */
    void limitResetEncoders() {
        if (pivotSwitchMin.get() || pivotSwitchMax.get()) {
            pivotEncoder.reset();
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
        if (isAtLimit(pivotEncoder.getDistance(), ArmConstants.kMinPivotAngle, ArmConstants.kMaxPivotAngle, rot))
            pivotMotor1.set(0.0);
        else 
            pivotMotor1.set(rot);

        if (getCurrentPoint().y >= ArmConstants.kMaxHeight)
            extensionMotor.set(-1.0);
        else if (isAtLimit(extensionEncoder.getPosition(), 0.0, ArmConstants.kArmMaxExtensionLength, ext))
            extensionMotor.set(0.0);
        else
            extensionMotor.set(ext);
    }

    /**
     * Sets the target position for the end effector to travel towards.
     * @param point A {@link Point} object representing a point in cartesian coordinate space to move to.
     */
    void setTargetPoint(Point point) {
        double angle = Math.atan(point.y/point.x);
        double distance = Math.sqrt(Math.pow(point.x, 2) + Math.pow(point.y, 2));
        
        // the length of the arm and end effector before any transformations
        double baseLength = ArmConstants.kArmRetractedLength + ArmConstants.kEndEffectorLength;

        if (distance - baseLength >= ArmConstants.kArmMaxExtensionLength || distance - baseLength < 0) {
            DriverStation.reportError("Not a valid distance; outside safe range.", null);
            return;
        }
        if (angle >= ArmConstants.kMaxPivotAngle || angle <= ArmConstants.kMinPivotAngle) {
            DriverStation.reportError("Not a valid pivot angle; outside safe range.", null);
            return;
        }
        if (point.y >= ArmConstants.kMaxHeight) {
            DriverStation.reportError("Cannot extend; Y coordinate is above max height.", null);
            return;
        }

        pivotGoal.position = angle;
        extensionGoal.position = distance - baseLength;
        
        pivotProfile = new TrapezoidProfile(pivotConstraints, pivotGoal, pivotCurrPoint);
        extensionProfile = new TrapezoidProfile(extensionConstraints, extensionGoal, extensionCurrPoint);

        pivotCurrPoint = pivotProfile.calculate(0.02);
        extensionCurrPoint = extensionProfile.calculate(0.02);

        pivotController.setReference(pivotCurrPoint.position, ControlType.kPosition, 0, 
            pivotFeedforward.calculate(pivotEncoder.getDistance(), 0)
        );

        extensionController.setReference(extensionCurrPoint.position, ControlType.kPosition, 0, 
            (extenstionFeedforward.calculate(extensionEncoder.getVelocity()) * Math.sin(pivotCurrPoint.position))
        );
    }

    /**
     * Gets the current position of the end effector in cartesian coordinate space.
     * @return A {@link Point} object representing the current position in cartesian coordinate space.
     */
    public Point getCurrentPoint() {
        Point point = new Point();

        // the length of the arm and end effector before any transformations
        double baseLength = ArmConstants.kArmRetractedLength + ArmConstants.kEndEffectorLength;
        
        // the length of the arm and end effector after transformations
        double extensionLength = baseLength + extensionEncoder.getPosition();
        
        point.x = extensionLength * Math.cos(pivotEncoder.getDistance());
        point.y = extensionLength * Math.sin(pivotEncoder.getDistance());
        return point;
    }

    @Override
    public void periodic() {
        updateShuffleboard(pivotMap, pivotController);
        updateShuffleboard(extensionMap, extensionController);
    }

    void updateShuffleboard(HashMap<String, Double> map, SparkMaxPIDController controller) {
        double p = SmartDashboard.getNumber("kP", map.get("kP"));
        double i = SmartDashboard.getNumber("kI", map.get("kI"));
        double d = SmartDashboard.getNumber("kD", map.get("kD"));
        double iz = SmartDashboard.getNumber("Izone", map.get("kIz"));

        if (p != map.get("kP")) controller.setP(p);
        if (i != map.get("kI")) controller.setI(i);
        if (d != map.get("kD")) controller.setD(d);
        if (iz != map.get("kIz")) controller.setIZone(iz);
        
        SmartDashboard.putNumber("kS", map.get("kS"));
        SmartDashboard.putNumber("kG", map.get("kG"));
        SmartDashboard.putNumber("kV", map.get("kV"));
        SmartDashboard.putNumber("kA", map.get("kA"));
    }
}
