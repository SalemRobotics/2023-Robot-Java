package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeRunCommand extends CommandBase{
    final Intake intake;
    final double speed;

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
