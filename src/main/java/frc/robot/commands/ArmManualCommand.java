package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmManualCommand extends CommandBase {
    final Arm arm;
    final DoubleSupplier ext, rot;

    public ArmManualCommand(Arm subsystem, DoubleSupplier extension, DoubleSupplier rotation) {
        arm = subsystem;
        ext = extension;
        rot = rotation;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setArmSpeeds(ext.getAsDouble(), rot.getAsDouble());
    }
}
