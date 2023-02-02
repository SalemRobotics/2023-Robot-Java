package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * This command allows default tank drive control over the drivetrain using command based control
 */
public class DefaultDrive extends CommandBase {
    private final Drivetrain drive;
    private final DoubleSupplier left;
    private final DoubleSupplier right;

    public DefaultDrive(Drivetrain subsystem, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed){
        drive = subsystem;
        left = leftSpeed;
        right = rightSpeed;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.arcadeDrive(left.getAsDouble(), right.getAsDouble());
    }
}
