package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivetrainConstants;

/**
 * Subsystem class for the drivetrain subsystem.
 * Allows tank drive control by a player-operated xbox controller.
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

    // differential drive object that controls the left and right motors
    DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    public Drivetrain() {
        /* 
         * the right motors should be inverted so that they do not 
         * drive in the opposite direction than the left motors 
         */
        rightMotors.setInverted(true);
    }

    /** 
     * Sets tank drive speeds for motors
     * @param left the speed for the left-side motors
     * @param right the speed for the right-side motors
     */
    public CommandBase arcadeDrive(DoubleSupplier left, DoubleSupplier right) {
        return run(
            () -> { // this is a lambda function to call arcade drive
                drive.arcadeDrive(left.getAsDouble(), -right.getAsDouble(), true);
            }
        );
    }
}
