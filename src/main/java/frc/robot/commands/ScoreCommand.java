package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ArmPresets;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class ScoreCommand extends ParallelRaceGroup {
    final Arm arm;
    final Intake intake;

    public ScoreCommand(Arm arm, Intake intake, ArmPresets cubePreset, ArmPresets conePreset) {
        this.arm = arm;
        this.intake = intake;
        addCommands(
            this.arm.setTargetPoint(cubePreset, conePreset),
            new SequentialCommandGroup(
                new WaitCommand(1.0),
                this.intake.intakeRun(IntakeConstants.kIntakeOutSpeed)
            ),
            new WaitCommand(1.5)
        );
    }
}
