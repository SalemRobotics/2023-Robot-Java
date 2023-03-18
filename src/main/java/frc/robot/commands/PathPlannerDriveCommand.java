package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class PathPlannerDriveCommand extends CommandBase {
    final Drivetrain drive;
    final SendableChooser<Command> m_chooser = new SendableChooser<>();
    final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(DrivetrainConstants.trackWidthMeters);

    public PathPlannerDriveCommand(Drivetrain drive) {
        m_chooser.setDefaultOption(DrivetrainConstants.kBottomPath, createPathCommand(DrivetrainConstants.kBottomPath));
        m_chooser.addOption(DrivetrainConstants.kBottomPathOneScore, createPathCommand(DrivetrainConstants.kBottomPathOneScore));
        m_chooser.addOption(DrivetrainConstants.kBottomPathNoEngage, createPathCommand(DrivetrainConstants.kBottomPathNoEngage));
        m_chooser.addOption(DrivetrainConstants.kTopPathNoEngage, createPathCommand(DrivetrainConstants.kTopPathNoEngage));
        m_chooser.addOption(DrivetrainConstants.kTopPath, createPathCommand(DrivetrainConstants.kTopPath));
        m_chooser.addOption(DrivetrainConstants.kTopPathOneScore, createPathCommand(DrivetrainConstants.kTopPathOneScore));
        m_chooser.addOption(DrivetrainConstants.kBottomPathOne, createPathCommand(DrivetrainConstants.kBottomPathOne));
        SmartDashboard.putData(m_chooser);
        this.drive=drive;
        addRequirements(this.drive);
    }

    Command createPathCommand(String pathName) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(4, 2));
    
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("score", new PrintCommand("Passed score"));
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
}
