package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem class for the drivetrain subsystem
 */
public class Drivetrain extends SubsystemBase {
    MotorControllerGroup leftMotors = new MotorControllerGroup(
        new CANSparkMax(Constants.DrivetrainConstants.kLeftFrontPort, MotorType.kBrushless),
        new CANSparkMax(Constants.DrivetrainConstants.kLeftBackPort, MotorType.kBrushless),
        new CANSparkMax(Constants.DrivetrainConstants.kLeftTopPort, MotorType.kBrushless)
    );

    MotorControllerGroup rightMotors = new MotorControllerGroup(
        new CANSparkMax(Constants.DrivetrainConstants.kRightFrontPort, MotorType.kBrushless),
        new CANSparkMax(Constants.DrivetrainConstants.kRightBackPort, MotorType.kBrushless),
        new CANSparkMax(Constants.DrivetrainConstants.kRightTopPort, MotorType.kBrushless)
    );

    DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

}
