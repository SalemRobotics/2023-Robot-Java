package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

/**
 * Moves the position of the {@link Arm} subsystem using manual operator controls. <p>
 * Gives the ability to control both rotation and extension of the arm.
 */
public class ArmManualCommand extends CommandBase {
    final Arm arm;
    final DoubleSupplier ext, rot;

    /**
     * Constructs an {@link ArmManualCommand} object.
     * @param subsystem The {@link Arm} subsystem object to reference.
     * @param extension The extension speed.
     * @param rotation The rotation speed.
     */
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
