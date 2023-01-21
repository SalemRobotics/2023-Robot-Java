package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    CANSparkMax motor = new CANSparkMax(IntakeConstants.kMotorPort, MotorType.kBrushless);

    /**
     * Sets the speed of the intake motor.
     * @param speed The speed the motor will run, from -1.0 to 1.0
     */
    public void setMotor(double speed) {
        motor.set(speed);
    }
}
