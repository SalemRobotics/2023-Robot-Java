// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.XBConstants;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private final Drivetrain robotDrive = new Drivetrain();
  
  private final XboxController driverController = new XboxController(XBConstants.drivePort);
  private final XboxController operatorController = new XboxController(XBConstants.opPort);

  public RobotContainer() {
    configureBindings();

    robotDrive.setDefaultCommand(
      robotDrive.arcadeDrive(driverController::getLeftY, driverController::getRightX)
    );
  }

  private void configureBindings() {}
    
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
