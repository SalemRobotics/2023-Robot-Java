// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakePresetCommand;
import frc.robot.constants.ArmPresets;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.XBConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  final XboxController driverController = new XboxController(XBConstants.drivePort);
  final XboxController operatorController = new XboxController(XBConstants.opPort);
  
  final Arm arm = new Arm();
  final Intake intake = new Intake();

  public RobotContainer() {
    configureBindings();

    // Uses joysticks to control the rotation and extension of the arm.
    // Left stick: Extension, Right stick: Rotation
    arm.setDefaultCommand(
      arm.setArmSpeeds(driverController::getLeftY, driverController::getRightY)
    );
  }

  private void configureBindings() {
    /* Driver Controller */
      // Set the arm to the intaking position and run the intake inwards
    new JoystickButton(driverController, Button.kRightBumper.value)
    .whileTrue(new IntakePresetCommand(arm, intake, IntakeConstants.kIntakeInSpeed))
    .onFalse(arm.setTargetPoint(ArmPresets.DEFAULT));

      // Set the arm to the intaking position and run the intake outwards, to eject the game piece
    new JoystickButton(driverController, Button.kLeftBumper.value)
    .whileTrue(new IntakePresetCommand(arm, intake, IntakeConstants.kIntakeOutSpeed))
    .onFalse(arm.setTargetPoint(ArmPresets.DEFAULT));

    /* Operator Controller */
      // Bindings for transforming the arm towards a preset point; 
      // X:Default, A:Low goal, B:Mid goal, Y:High goal
    new JoystickButton(operatorController, Button.kX.value)
    .onTrue(arm.setTargetPoint(ArmPresets.DEFAULT));

    new JoystickButton(operatorController, Button.kA.value)
    .onTrue(arm.setTargetPoint(ArmPresets.LOW_GOAL));

    new JoystickButton(operatorController, Button.kB.value)
    .onTrue(arm.setTargetPoint(ArmPresets.MID_GOAL));

    new JoystickButton(operatorController, Button.kY.value)
    .onTrue(arm.setTargetPoint(ArmPresets.HIGH_GOAL));

      // Used to release the game piece without moving the arm
    new JoystickButton(operatorController, Button.kRightBumper.value)
    .whileTrue(intake.intakeRun(IntakeConstants.kIntakeOutSpeed));

      // Used to re-intake the game piece, if necessary
    new JoystickButton(operatorController, Button.kLeftBumper.value)
    .whileTrue(intake.intakeRun(IntakeConstants.kIntakeInSpeed));
  }
    
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
