// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.TrajectoryDriveCommand;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.XBConstants;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private final Drivetrain robotDrive = new Drivetrain();
  
  private final XboxController driverController = new XboxController(XBConstants.drivePort);
  private final XboxController operatorController = new XboxController(XBConstants.opPort);

  private static final String kBottomPathNoEngage = "Bottom Path No Engage";
  private static final String kBottomPathOneScore = "Bottom Path Two Score";
  private static final String kBottomPath = "Bottom Path";
  private static final String kTopPathNoEngage = "Top Path No Engage";
  private static final String kTopPathOneScore = "Top Path Two Score";
  private static final String kTopPath = "Top Path";
  private static final String kBottomPathOne = "Bottom Path 1";
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    robotDrive.setDefaultCommand(
      robotDrive.arcadeDrive(driverController::getLeftY, driverController::getRightX)
    );

    m_chooser.setDefaultOption(kBottomPath, createPathCommand(kBottomPath));
    m_chooser.addOption(kBottomPathOneScore, createPathCommand(kBottomPathOneScore));
    m_chooser.addOption(kBottomPathNoEngage, createPathCommand(kBottomPathNoEngage));
    m_chooser.addOption(kTopPathNoEngage, createPathCommand(kTopPathNoEngage));
    m_chooser.addOption(kTopPath, createPathCommand(kTopPath));
    m_chooser.addOption(kTopPathOneScore, createPathCommand(kTopPathOneScore));
    m_chooser.addOption(kBottomPathOne, createPathCommand(kBottomPathOne));

    SmartDashboard.putData(m_chooser);

  }
  

  private void configureBindings() {}

  private Command createPathCommand(String pathName) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(4, 2));

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("score", new PrintCommand("Passed score"));
    eventMap.put("intake", new PrintCommand("Passed intake"));
    eventMap.put("engage", new PrintCommand("Passed engage"));
    eventMap.put("Turn 180", robotDrive.turn180());
    eventMap.put("BPath 2", getAutonomousCommand());

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
  
    return autoBuilder.fullAuto(pathGroup);
  }
    
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
