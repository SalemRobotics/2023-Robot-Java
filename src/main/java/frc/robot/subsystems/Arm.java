package frc.robot.subsystems;

import org.opencv.core.Point;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotProperties;
import frc.robot.constants.ArmConstants;

public class Arm extends SubsystemBase {
    CANSparkMax pivotMotor1 = new CANSparkMax(ArmConstants.kPivotPort1, MotorType.kBrushless);
    CANSparkMax pivotMotor2 = new CANSparkMax(ArmConstants.kPivotPort2, MotorType.kBrushless);

    CANSparkMax extensionMotor = new CANSparkMax(ArmConstants.kExtensionPort, MotorType.kBrushless);

    AnalogEncoder pivotEncoder = new AnalogEncoder(ArmConstants.kPivotEncoderPort);
    Encoder extensionEncoder = new Encoder(ArmConstants.kExtensionEncoderSourceA, ArmConstants.kExtensionEncoderSourceB);

    SparkMaxPIDController pivotController;
    SparkMaxPIDController extensionController;

    double kPivotP, kPivotI, kPivotD,
        kPivotIz, kPivotFF, kPivotMaxOutput, kPivotMinOutput,
        kPivotMaxVel, kPivotMinVel, kPivotMaxAcc, kPivotAllowedErr;

    double kExtP, kExtI, kExtD, 
        kExtIz, kExtFF, kExtMaxOutput, kExtMinOutput,
        kExtMaxVel, kExtMinVel, kExtMaxAcc, kExtAllowedErr;

    public Arm() {
        pivotMotor2.follow(pivotMotor1);

        extensionEncoder.setDistancePerPulse(0); // TODO: distancePerPulse = pulley circum / pulses per rotation

        // Pivot Loop Consts 
        // TODO: Profile arm to get the below values. Currently default.
        kPivotP = 0.1; 
        kPivotI = 1e-4;
        kPivotD = 1; 
        kPivotIz = 0; 
        kPivotFF = 0; 
        kPivotMaxOutput = 1; 
        kPivotMinOutput = -1;

        pivotController = pivotMotor1.getPIDController();
        pivotController.setP(kPivotP);
        pivotController.setI(kPivotI);
        pivotController.setD(kPivotD);
        pivotController.setIZone(kPivotIz);
        pivotController.setFF(kPivotFF);
        pivotController.setOutputRange(kPivotMinOutput, kPivotMaxOutput);

        // Extension Loop Consts
        // TODO: Profile arm to get the below values. Currently default.
        kExtP = 0.1; 
        kExtI = 1e-4;
        kExtD = 1; 
        kExtIz = 0; 
        kExtFF = 0; 
        kExtMaxOutput = 1; 
        kExtMinOutput = -1;

        extensionController = extensionMotor.getPIDController();
        extensionController.setP(kExtP);
        extensionController.setI(kExtI);
        extensionController.setD(kExtD);
        extensionController.setIZone(kExtIz);
        extensionController.setFF(kExtFF);
        extensionController.setOutputRange(kExtMinOutput, kExtMaxOutput);

        // RobotProperties.loadPIDConstants("PivotPID", pivotController);
        // RobotProperties.loadPIDConstants("ExtensionPID", extensionController);
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
     * Manually set the speeds for the extension and pivot of the arm subsystem
     * @param extension the speed to extend at
     * @param rotation the speed to pivot at
     */
    public void setArmSpeeds(double extension, double rotation) {
        if (isAtLimit(pivotEncoder.getAbsolutePosition(), ArmConstants.kMinPivotAngle, ArmConstants.kMaxPivotAngle, rotation))
            pivotController.setReference(0.0, ControlType.kVelocity);
        else 
            pivotController.setReference(rotation, ControlType.kVelocity);

        if (getCurrentPoint().y >= ArmConstants.kMaxHeight)
            extensionController.setReference(-1.0, ControlType.kVelocity);
        else if (isAtLimit(extensionEncoder.getDistance(), 0.0, ArmConstants.kArmMaxExtensionLength, extension))
            extensionController.setReference(0.0, ControlType.kVelocity);
        else
            extensionController.setReference(extension, ControlType.kVelocity);
    }

    /**
     * Sets the target position for the end effector to travel towards.
     * @param point A Point object represnting a point in local coordinate space to move to.
     */
    public void setTargetPoint(Point point) {
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

        pivotController.setReference(angle, ControlType.kSmartMotion);
        extensionController.setReference(distance - baseLength, ControlType.kSmartMotion);
    }

    /**
     * Gets the current position of the end effector in local coordinate space.
     * @return A point object representing the current position in local coordinate space.
     */
    public Point getCurrentPoint() {
        Point point = new Point();

        // the length of the arm and end effector before any transformations
        double baseLength = ArmConstants.kArmRetractedLength + ArmConstants.kEndEffectorLength;
        
        // the length of the arm and end effector after transformations
        double extensionLength = baseLength + extensionEncoder.getDistance();
        
        point.x = extensionLength * Math.cos(pivotEncoder.getAbsolutePosition());
        point.y = extensionLength * Math.sin(pivotEncoder.getAbsolutePosition());
        return point;
    }

    @Override
    public void periodic() {
        updatePivotConsts();
        updateExtensionConsts();
    }

    /**
     * TODO: once satisfied with constants, read them from a file instead of smart dashboard
     */
    void updatePivotConsts() {
        double p = SmartDashboard.getNumber("Error Multiplier (P)", kPivotP);
        double i = SmartDashboard.getNumber("Sum Error (I)", kPivotI);
        double d = SmartDashboard.getNumber("Slope Error (D)", kPivotD);
        double iz = SmartDashboard.getNumber("Integral Effective Error Range (IZone)", kPivotIz);
        double ff = SmartDashboard.getNumber("Control Loop Gain (Feed Forward)", kPivotFF);
        double min = SmartDashboard.getNumber("Min Output", kPivotMinOutput);
        double max = SmartDashboard.getNumber("Max Output", kPivotMaxOutput);
        double minV = SmartDashboard.getNumber("Min Velocity", kPivotMinVel);
        double maxV = SmartDashboard.getNumber("Max Velocity", kPivotMaxVel);
        double maxA = SmartDashboard.getNumber("Max Acceleration", kPivotMaxAcc);
        double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", kPivotAllowedErr); 

        if (p != kPivotP) { pivotController.setP(p); kPivotP = p; }
        if (i != kPivotI) { pivotController.setI(i); kPivotI = i; }
        if (d != kPivotD) { pivotController.setD(d); kPivotD = d; }
        if (iz != kPivotIz) { pivotController.setIZone(iz); kPivotIz = iz; }
        if (ff != kPivotFF) { pivotController.setFF(p); kPivotFF = ff; }
        if ((max != kPivotMaxOutput) || (min != kPivotMinOutput)) {
            pivotController.setOutputRange(min, max);
            kPivotMinOutput = min; kPivotMaxOutput = max;
        }
        if (maxV != kPivotMaxVel) { pivotController.setSmartMotionMaxVelocity(maxV, 0); kPivotMaxVel = maxV; }
        if (minV != kPivotMinVel) { pivotController.setSmartMotionMinOutputVelocity(minV, 0); kPivotMinVel = minV; }
        if (maxA != kPivotMaxAcc) { pivotController.setSmartMotionMaxAccel(maxA, 0); kPivotMaxAcc = maxA; }
        if (allE != kPivotAllowedErr) { pivotController.setSmartMotionAllowedClosedLoopError(allE, 0); kPivotAllowedErr = allE; }
    }

    /**
     * TODO: once satisfied with constants, read them from a file instead of smart dashboard
     */
    void updateExtensionConsts() {
        double p =SmartDashboard.getNumber("Error Multiplier (P)", kExtP);
        double i =SmartDashboard.getNumber("Sum Error (I)", kExtI);
        double d =SmartDashboard.getNumber("Slope Error (D)", kExtD);
        double iz =SmartDashboard.getNumber("Integral Effective Error Range (IZone)", kExtIz);
        double ff =SmartDashboard.getNumber("Control Loop Gain (Feed Forward)", kExtFF);
        double min =SmartDashboard.getNumber("Min Output", kExtMinOutput);
        double max =SmartDashboard.getNumber("Max Output", kExtMaxOutput);
        double minV = SmartDashboard.getNumber("Min Velocity", kExtMinVel);
        double maxV = SmartDashboard.getNumber("Max Velocity", kExtMaxVel);
        double maxA = SmartDashboard.getNumber("Max Acceleration", kExtMaxAcc);
        double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", kExtAllowedErr); 

        if (p != kExtP) { extensionController.setP(p); kExtP = p; }
        if (i != kExtI) { extensionController.setI(i); kExtI = i; }
        if (d != kExtD) { extensionController.setD(d); kExtD = d; }
        if (iz != kExtIz) { extensionController.setIZone(iz); kExtIz = iz; }
        if (ff != kExtFF) { extensionController.setFF(p); kExtFF = ff; }
        if ((max != kExtMaxOutput) || (min != kExtMinOutput)) {
            extensionController.setOutputRange(min, max);
            kExtMinOutput = min; kExtMaxOutput = max;
        }
        if (maxV != kExtMaxVel) { extensionController.setSmartMotionMaxVelocity(maxV, 0); kExtMaxVel = maxV; }
        if (minV != kExtMinVel) { extensionController.setSmartMotionMinOutputVelocity(minV, 0); kExtMinVel = minV; }
        if (maxA != kExtMaxAcc) { extensionController.setSmartMotionMaxAccel(maxA, 0); kExtMaxAcc = maxA; }
        if (allE != kExtAllowedErr) { extensionController.setSmartMotionAllowedClosedLoopError(allE, 0); kExtAllowedErr = allE; }
    }
}
