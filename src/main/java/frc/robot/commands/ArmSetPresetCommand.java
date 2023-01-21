package frc.robot.commands;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ArmPresets;
import frc.robot.subsystems.Arm;

public class ArmSetPresetCommand extends CommandBase {
    final Arm arm;
    final Point point;
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
