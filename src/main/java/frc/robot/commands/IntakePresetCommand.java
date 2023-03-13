package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.ArmPresets;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StatusLED;

/**
 * Moves the position of the {@link Arm} subsystem to intake position while running the 
 * {@link Intake} subsystem at a desired speed simultaneously. 
 */
public class IntakePresetCommand extends ParallelCommandGroup {

    /**
     * Constructs an {@link IntakePresetCommand} object.
     * @param armSubsystem an {@link Arm} subsystem object.
     * @param intakeSubsystem an {@link Intake} subsystem object.
     * @param speed the desired speed to run the intake at.
     */
    public IntakePresetCommand(Arm armSubsystem, Intake intakeSubsystem, StatusLED led, double speed) {
        addCommands(
            armSubsystem.setTargetPoint(ArmPresets.INTAKE),
            intakeSubsystem.intakeRun(speed)
        );
    }
}
