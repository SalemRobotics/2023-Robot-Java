package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    MotorControllerGroup pivotMotors = new MotorControllerGroup(
        new CANSparkMax(Constants.ArmConstants.pivotPort1, MotorType.kBrushless),
        new CANSparkMax(Constants.ArmConstants.pivotPort2, MotorType.kBrushless)
    );

    CANSparkMax extensionMotor = new CANSparkMax(Constants.ArmConstants.encoderPort, MotorType.kBrushless);
    AnalogEncoder absEncoder = new AnalogEncoder(Constants.ArmConstants.encoderPort);

    public void setPivotSpeed(double power) {
        pivotMotors.set(power);
    }

    public void setExtendSpeed(double power) {
        extensionMotor.set(power);
    }
}
