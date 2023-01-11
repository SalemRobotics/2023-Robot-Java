package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;

public class Drivetrain {
    MotorControllerGroup leftMotors = new MotorControllerGroup(
        new CANSparkMax(Constants.DrivetrainConstants.kLeftFrontPort, MotorType.kBrushless),
        new CANSparkMax(Constants.DrivetrainConstants.kLeftBackPort, MotorType.kBrushless),
        new CANSparkMax(Constants.DrivetrainConstants.kLeftTopPort, MotorType.kBrushless));

}
