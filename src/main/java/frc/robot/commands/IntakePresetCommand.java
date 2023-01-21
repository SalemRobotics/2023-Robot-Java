package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.ArmPresets;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class IntakePresetCommand extends ParallelCommandGroup {
    public IntakePresetCommand(Arm armSubsystem, Intake intakeSubsystem, double speed) {
        addCommands(
            new ArmSetPresetCommand(armSubsystem, ArmPresets.INTAKE),
            new IntakeRunCommand(intakeSubsystem, speed)
        );
    }
}
