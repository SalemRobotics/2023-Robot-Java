package frc.robot.commands;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ArmPresets;
import frc.robot.subsystems.Arm;

/**
 * Moves the position of the {@link Arm} subsystem to the desired preset position.
 */
public class ArmSetPresetCommand extends CommandBase {
    final Arm arm;
    final Point point;

    /**
     * Constructs an {@link ArmSetPresetCommand} object.
     * @param subsystem The {@link Arm} subsystem object to reference. 
     * @param preset The desired {@link ArmPresets} preset position.
     */
    public ArmSetPresetCommand(Arm subsystem, ArmPresets preset) {
        arm = subsystem;
        point = preset.value;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
       arm.setTargetPoint(point); 
    }
}
