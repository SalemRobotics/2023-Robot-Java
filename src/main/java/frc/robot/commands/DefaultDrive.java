package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DefaultDrive extends CommandBase {
    private final Drivetrain drive;
    private final DoubleSupplier forward;
    private final DoubleSupplier rotation;
    private final boolean reversed;

    public DefaultDrive(Drivetrain subsystem, DoubleSupplier fwd, DoubleSupplier rot, boolean reversed){
        drive = subsystem;
        forward = fwd;
        rotation = rot;
        this.reversed = reversed;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (reversed) {
            drive.arcadeDrive(-forward.getAsDouble(), rotation.getAsDouble());
        } else {
            drive.arcadeDrive(forward.getAsDouble(), rotation.getAsDouble());
        }
    }
}
