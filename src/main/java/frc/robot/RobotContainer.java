// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.ArmPresets;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.XBConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StatusLED;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.PathPlannerDriveCommand;

public class RobotContainer {
  final XboxController driverController = new XboxController(XBConstants.kDriverPort);
  final XboxController operatorController = new XboxController(XBConstants.kOperatorPort);
  
  final Drivetrain drivetrain = new Drivetrain();
  final Arm arm = new Arm();
  final Intake intake = new Intake();
  final PathPlannerDriveCommand pathPlannerCommand = new PathPlannerDriveCommand(drivetrain);

  final StatusLED led = new StatusLED();

  public RobotContainer() {
    configureBindings();

    drivetrain.setDefaultCommand(
      drivetrain.arcadeDrive(driverController::getLeftX, driverController::getLeftY)
    );

    // Uses joysticks to control the rotation and extension of the arm.
    // Left stick: Extension, Right stick: Rotation
    arm.setDefaultCommand(
      arm.setTargetPoint(ArmPresets.DEFAULT, ArmPresets.DEFAULT)
    );

    // arm.setDefaultCommand(
    //   arm.setArmSpeeds(operatorController::getLeftY, operatorController::getRightY)
    // );

    led.setDefaultCommand(led.solidTeamColor());
  }
  
  private void configureBindings() {

    /* Driver Controller */
      // Set Cube mode and blink purple
    new JoystickButton(driverController, Button.kRightBumper.value)
    .toggleOnTrue(
      new InstantCommand(() -> { Arm.isConeMode = false; })
      .alongWith(led.gamepieceSolidColor())
    );

      // Set Cone mode and blink yellow
    new JoystickButton(driverController, Button.kLeftBumper.value)
    .toggleOnTrue(
      new InstantCommand(() -> { Arm.isConeMode = true; })
      .alongWith(led.gamepieceSolidColor())
    );

    /* Operator Controller */
      // Hold...
      // A = low goal
      // B = Mid goal
      // Y = High goal
      // X = Manual Control
    new JoystickButton(operatorController, Button.kA.value)
    .whileTrue(arm.setTargetPoint(ArmPresets.CUBE_LOW_GOAL, ArmPresets.CONE_LOW_GOAL));

    new JoystickButton(operatorController, Button.kB.value)
    .whileTrue(arm.setTargetPoint(ArmPresets.CUBE_MID_GOAL, ArmPresets.CONE_MID_GOAL));

    new JoystickButton(operatorController, Button.kY.value)
    .whileTrue(arm.setTargetPoint(ArmPresets.CUBE_HIGH_GOAL, ArmPresets.CONE_HIGH_GOAL));

    new JoystickButton(operatorController, Button.kX.value)
    .whileTrue(arm.setArmSpeeds(operatorController::getLeftY, operatorController::getRightY));
    
      // Snapshot encoder positions
    // new JoystickButton(operatorController, Button.kA.value)
    // .onTrue(arm.snapshotEncoderPosition());

      // Intake: will run intake
    new JoystickButton(operatorController, Button.kRightBumper.value)
    .whileTrue(intake.intakeRun(IntakeConstants.kIntakeInSpeed));

      // Blink green when gamepiece is aquired, to notify the operator to stop. Also rumbles controller.
    intake.hitCurrentLimit.whileTrue(
      led.overCurrentBlink()
      .alongWith(new RunCommand(() -> { 
        driverController.setRumble(RumbleType.kBothRumble, 1); 
        operatorController.setRumble(RumbleType.kBothRumble, 1);
      }))
    ).onFalse(led.gamepieceSolidColor()
    .alongWith(new RunCommand(() -> {
      driverController.setRumble(RumbleType.kBothRumble, 0);
      operatorController.setRumble(RumbleType.kBothRumble, 0);
    })));
    
      // Release game piece
    new JoystickButton(operatorController, Button.kLeftBumper.value)
    .whileTrue(intake.intakeRun(IntakeConstants.kIntakeOutSpeed));
  }
    
  public Command getAutonomousCommand() {
    return pathPlannerCommand.getCommand();
  }

  public Command getDisabledCommand() {
    return led.breathTeamColor();
  }
}
