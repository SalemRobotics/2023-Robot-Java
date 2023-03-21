package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class PathPlannerDriveCommand extends CommandBase {
    final Drivetrain drive;
    final Arm arm;
    final Intake intake;
    final SendableChooser<Command> m_chooser = new SendableChooser<>();
    final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(DrivetrainConstants.trackWidthMeters);

    public PathPlannerDriveCommand(Drivetrain drive, Arm arm, Intake intake) {
        m_chooser.setDefaultOption(DrivetrainConstants.kBottomPath, createPathCommand(DrivetrainConstants.kBottomPath));
        m_chooser.addOption(DrivetrainConstants.kBottomPathOneScore, createPathCommand(DrivetrainConstants.kBottomPathOneScore));
        m_chooser.addOption(DrivetrainConstants.kBottomPathNoEngage, createPathCommand(DrivetrainConstants.kBottomPathNoEngage));
        m_chooser.addOption(DrivetrainConstants.kTopPathNoEngage, createPathCommand(DrivetrainConstants.kTopPathNoEngage));
        m_chooser.addOption(DrivetrainConstants.kTopPath, createPathCommand(DrivetrainConstants.kTopPath));
        m_chooser.addOption(DrivetrainConstants.kTopPathOneScore, createPathCommand(DrivetrainConstants.kTopPathOneScore));
        m_chooser.addOption(DrivetrainConstants.kBottomPathOne, createPathCommand(DrivetrainConstants.kBottomPathOne));
        SmartDashboard.putData(m_chooser);
        this.drive=drive;
        this.arm=arm;
        this.intake=intake;
        addRequirements(this.drive);
    }

    Command createPathCommand(String pathName) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(4, 2));
    
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("score", new PrintCommand("Passed Score_default"));
        eventMap.put("score_high", new PrintCommand("Passed score_high"));
        eventMap.put("score_mid", new PrintCommand("Passed score_mid"));
        eventMap.put("score_low", new PrintCommand("Passed score_low"));
        eventMap.put("intake", new PrintCommand("Passed intake"));
        eventMap.put("engage", new PrintCommand("Passed engage"));
        eventMap.put("Turn 180", drive.turn180());
        eventMap.put("BPath 2", getCommand());
    
        RamseteAutoBuilder autoBuilder =  new RamseteAutoBuilder(
          drive::getPose, 
          drive::resetOdometry, 
          new RamseteController(DrivetrainConstants.kRamseteB, DrivetrainConstants.kRamseteZeta), 
          driveKinematics, 
          new SimpleMotorFeedforward(DrivetrainConstants.ksVolts, DrivetrainConstants.kvVoltSecondsPerMeter, DrivetrainConstants.kaVoltSecondsSquaredPerMeter), 
          drive::getWheelSpeeds, 
          new PIDConstants(DrivetrainConstants.kPDriveVel, DrivetrainConstants.kIDriveVel, DrivetrainConstants.KDDriveVel), 
          drive::tankDriveVolts, 
          eventMap, 
          true, 
          drive
        );
      
        return autoBuilder.fullAuto(pathGroup);
      }

    public Command getCommand() {
        return m_chooser.getSelected();
    }

    public Command testCommand() {
      var feedForward = new SimpleMotorFeedforward(
        DrivetrainConstants.ksVolts, 
        DrivetrainConstants.kvVoltSecondsPerMeter, 
        DrivetrainConstants.kaVoltSecondsSquaredPerMeter
      );

      var voltageConstraint = new DifferentialDriveVoltageConstraint(feedForward, driveKinematics, 10);
      
      var config = new TrajectoryConfig(
        DrivetrainConstants.kMaxSpeedMetersPerSecond, 
        DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(driveKinematics)
      .addConstraint(voltageConstraint);

      var trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)), 
        new Pose2d(3, 0, new Rotation2d(0)), 
        config
      );

      return new RamseteCommand(
        trajectory, 
        drive::getPose, 
        new RamseteController(DrivetrainConstants.kRamseteB, DrivetrainConstants.kRamseteZeta), 
        feedForward, 
        driveKinematics, 
        drive::getWheelSpeeds, 
        new PIDController(DrivetrainConstants.kPDriveVel, DrivetrainConstants.kIDriveVel, DrivetrainConstants.KDDriveVel), 
        new PIDController(DrivetrainConstants.kPDriveVel, DrivetrainConstants.kIDriveVel, DrivetrainConstants.KDDriveVel), 
        drive::tankDriveVolts, 
        drive
      );
    }
}
