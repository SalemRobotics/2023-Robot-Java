package frc.robot.commands;

import java.util.HashMap;
import java.util.List;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ArmPresets;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.IntakeConstants;
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
      this.drive=drive;
      this.arm=arm;
      this.intake=intake;
      m_chooser.setDefaultOption(DrivetrainConstants.k2ScoreCube, createPathCommand(DrivetrainConstants.k2ScoreCube));
      m_chooser.addOption(DrivetrainConstants.kCharger, createPathCommand(DrivetrainConstants.kCharger));
      m_chooser.addOption(DrivetrainConstants.kChargerMobilityPath, createPathCommand(DrivetrainConstants.kChargerMobilityPath));
      m_chooser.addOption(DrivetrainConstants.k2ScoreCube, createPathCommand(DrivetrainConstants.k2ScoreCube));
      m_chooser.addOption(DrivetrainConstants.k2ScoreCubeEngage, createPathCommand(DrivetrainConstants.k2ScoreCubeEngage));
      m_chooser.addOption(DrivetrainConstants.k3ScoreCube, createPathCommand(DrivetrainConstants.k3ScoreCube));
      m_chooser.addOption(DrivetrainConstants.k3ScoreCubeEngage, createPathCommand(DrivetrainConstants.k3ScoreCubeEngage));
      m_chooser.addOption(DrivetrainConstants.k25ScoreCube, createPathCommand(DrivetrainConstants.k25ScoreCube));
      m_chooser.addOption(DrivetrainConstants.k25ScoreCubeEngage, createPathCommand(DrivetrainConstants.k25ScoreCubeEngage));
      m_chooser.addOption(DrivetrainConstants.k1ScoreHigh, createPathCommand(DrivetrainConstants.k1ScoreHigh));
      m_chooser.addOption(DrivetrainConstants.k1ScoreLow, createPathCommand(DrivetrainConstants.k1ScoreLow));
      m_chooser.addOption(DrivetrainConstants.kBump1ScoreMobility, createPathCommand(DrivetrainConstants.kBump1ScoreMobility));
      m_chooser.addOption(DrivetrainConstants.kBump2Score, createPathCommand(DrivetrainConstants.kBump2Score));
      m_chooser.addOption(DrivetrainConstants.kBump3Score, createPathCommand(DrivetrainConstants.kBump3Score));
      m_chooser.addOption(DrivetrainConstants.kCharger1ScoreMobility, createPathCommand(DrivetrainConstants.kCharger1ScoreMobility));
      m_chooser.addOption(DrivetrainConstants.kFlat1ScoreMobility, createPathCommand(DrivetrainConstants.kFlat1ScoreMobility));

      SmartDashboard.putData(m_chooser);
      addRequirements(drive, arm, intake);
    }

    Command createPathCommand(String pathName) {
        List<PathPlannerTrajectory> pathGroup = 
        PathPlanner.loadPathGroup(
          pathName, 
          DrivetrainConstants.kMaxSpeedMetersPerSecond,
          DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared,
          true
        );
    
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("score_low", new ScoreCommand(arm, intake, ArmPresets.CUBE_LOW_GOAL, ArmPresets.CONE_LOW_GOAL));
        eventMap.put("score_mid", new ScoreCommand(arm, intake, ArmPresets.CUBE_MID_GOAL, ArmPresets.CONE_MID_GOAL));
        eventMap.put("score_high", new ScoreCommand(arm, intake, ArmPresets.CUBE_HIGH_GOAL, ArmPresets.CONE_HIGH_GOAL));
        eventMap.put("set_cone_mode", new InstantCommand( () -> { Arm.isConeMode = true; } ));
        eventMap.put("set_cube_mode", new InstantCommand( () -> { Arm.isConeMode = false; } ));
        eventMap.put("arm_default", arm.setTargetPoint(ArmPresets.DEFAULT, ArmPresets.DEFAULT));
        eventMap.put("arm_low", arm.setTargetPoint(ArmPresets.CUBE_LOW_GOAL, ArmPresets.CONE_LOW_GOAL));
        eventMap.put("arm_mid", arm.setTargetPoint(ArmPresets.CUBE_MID_GOAL, ArmPresets.CONE_MID_GOAL));
        eventMap.put("arm_high", arm.setTargetPoint(ArmPresets.CUBE_HIGH_GOAL, ArmPresets.CONE_HIGH_GOAL));
        eventMap.put("intake_in", intake.intakeRun(IntakeConstants.kIntakeInSpeed));
        eventMap.put("intake_out", intake.intakeRun(-1.0));
        eventMap.put("intake_stop", intake.intakeRun(0));
        eventMap.put("balance", drive.alignToCharger());
        eventMap.put("balance_taxi", drive.taxiOverCharger());
    
        RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
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
