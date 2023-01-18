package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivetrainConstants;

/**
 * Subsystem class for the drivetrain subsystem
 */
public class Drivetrain extends SubsystemBase {
    MotorControllerGroup leftMotors = new MotorControllerGroup(
        new CANSparkMax(DrivetrainConstants.kLeftFrontPort, MotorType.kBrushless),
        new CANSparkMax(DrivetrainConstants.kLeftBackPort, MotorType.kBrushless),
        new CANSparkMax(DrivetrainConstants.kLeftTopPort, MotorType.kBrushless)
    );

    MotorControllerGroup rightMotors = new MotorControllerGroup(
        new CANSparkMax(DrivetrainConstants.kRightFrontPort, MotorType.kBrushless),
        new CANSparkMax(DrivetrainConstants.kRightBackPort, MotorType.kBrushless),
        new CANSparkMax(DrivetrainConstants.kRightTopPort, MotorType.kBrushless)
    );

    DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    public Drivetrain() {
        rightMotors.setInverted(true);
    }
 
    /** 
     * sets volts to control motors at a set speed 
     */
    public void tankDriveVolts(Double speed) {
        rightMotors.set(speed);
        leftMotors.set(speed);
    }

    /** 
     * sets arcade drive for motors
     * fwd is Forward axis of drivetrain (Y axis)
     * rot is Axis of rotation of drivetrain (X axis) 
     */
    public void arcadeDrive(double fwd, double rot) {
        drive.arcadeDrive(-fwd, rot, true);
    }
}
