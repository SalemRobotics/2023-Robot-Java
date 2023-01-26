package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/**
 * Runs the {@link Intake} at a desired speed.
 */
public class IntakeRunCommand extends CommandBase{
    final Intake intake;
    final double speed;

    /**
     * Constructs an {@link IntakeRunCommand} object.
     * @param subsystem The {@link Intake} subsystem object to reference.
     * @param speed The desired speed to run the intake motor at.
     */
    public IntakeRunCommand(Intake subsystem, double speed) {
        intake = subsystem;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotor(0.0);
    }
}
