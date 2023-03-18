package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.IntakeConstants;

/**
 * The Intake subsystem allows control over a single NEO motor.
 */
public class Intake extends SubsystemBase {

    CANSparkMax motor = new CANSparkMax(IntakeConstants.kMotorPort, MotorType.kBrushless);
    public Trigger hitCurrentLimit = new Trigger(() -> { return motor.getOutputCurrent() > IntakeConstants.kCurrentLimit; });

    /**
     * Sets the speed of the intake motor. This command will stop 
     * @param speed The speed the motor will run, from -1.0 to 1.0
     */
    public CommandBase intakeRun(double speed) {
        return new FunctionalCommand(
            ()->{},

            // execute
            () -> { 
                double temp = Arm.isConeMode ? -speed : speed;
                motor.set(temp); 
            },
            
            // end
            isFinished -> { motor.set(0); },

            // isFinished?
            () -> false,

            // subsystem requirement
            this
        );
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Current", motor.getOutputCurrent());
    }
}
