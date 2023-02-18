package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

/**
 * The Intake subsystem allows control over a single NEO motor.
 */
public class Intake extends SubsystemBase {
    CANSparkMax motor = new CANSparkMax(IntakeConstants.kMotorPort, MotorType.kBrushless);
    DigitalInput gamePieceSensor = new DigitalInput(IntakeConstants.kBreakBeamSensorPort);

    /**
     * Sets the speed of the intake motor. This command will stop 
     * @param speed The speed the motor will run, from -1.0 to 1.0
     */
    public CommandBase intakeRun(double speed) {
        return new FunctionalCommand(
            ()->{},

            // execute
            () -> { motor.set(speed); },
            
            // end
            isFinished -> { motor.set(0.0); },

            // isFinished?
            () -> !gamePieceSensor.get(),

            // subsystem requirement
            this
        );
    }
}
