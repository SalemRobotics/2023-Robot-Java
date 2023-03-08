// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.TrajectoryDriveCommand;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.XBConstants;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private final Drivetrain robotDrive = new Drivetrain();
  
  private final XboxController driverController = new XboxController(XBConstants.drivePort);
  private final XboxController operatorController = new XboxController(XBConstants.opPort);

  private final Command fullAuto;

  public RobotContainer() {
    configureBindings();

    robotDrive.setDefaultCommand(
      robotDrive.arcadeDrive(driverController::getLeftY, driverController::getRightX)
    );

    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Bottom Path", new PathConstraints(4, 2));

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("score", new PrintCommand("Passed score"));
    eventMap.put("intake", new PrintCommand("Passed intake"));
    eventMap.put("engage", new PrintCommand("Passed engage"));

    RamseteAutoBuilder autoBuilder =  new RamseteAutoBuilder(
      robotDrive::getPose, 
      robotDrive::resetOdometry, 
      new RamseteController(DrivetrainConstants.kRamseteB, DrivetrainConstants.kRamseteZeta), 
      TrajectoryDriveCommand.driveKinematics, 
      new SimpleMotorFeedforward(0, 0, 0), 
      robotDrive::getWheelSpeeds, 
      new PIDConstants(0, 0, 0), 
      robotDrive::tankDriveVolts, 
      eventMap, 
      true, 
      robotDrive
    );
  
    fullAuto = autoBuilder.fullAuto(pathGroup);
  }
  

  private void configureBindings() {}
    
  public Command getAutonomousCommand() {
    return fullAuto;
  }
}
