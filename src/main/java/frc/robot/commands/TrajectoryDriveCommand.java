package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * the TrajectoryDriveCommand class hosts the methods for getting the trajectory command and creates
 * the driveKinematics object
 */
public class TrajectoryDriveCommand extends CommandBase{  
    
    final DifferentialDriveKinematics driveKinematics = 
    new DifferentialDriveKinematics(DrivetrainConstants.trackWidthMeters);

    /**
     * uses data from the drivetrain to calculate and return the trajectory
     * @param drive  The drive paramter is the drivetrain subsytem and holds the values it is associated with
     * @return returns the ramsete trajectroy command
     */
    public Command getCommand(Drivetrain drive) {
        SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(
            DrivetrainConstants.ksVolts,
            DrivetrainConstants.kvVoltSecondsPerMeter,
            DrivetrainConstants.kaVoltSecondsSquaredPerMeter
        );
       
        DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            feedforward,
            driveKinematics,
            10
        ); 

        TrajectoryConfig config = 
        new TrajectoryConfig(
            DrivetrainConstants.kMaxSpeedMetersPerSecond,
            DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared
        )
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(driveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

                // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config
        );

        RamseteCommand ramseteCommand =
            new RamseteCommand(
            exampleTrajectory,
            drive::getPose,
            new RamseteController(DrivetrainConstants.kRamseteB, DrivetrainConstants.kRamseteZeta),
            feedforward,
            driveKinematics,
            drive::getWheelSpeeds,
            new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
            new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            drive::tankDriveVolts,
            drive
         );

         // Reset odometry to the starting pose of the trajectory .
        drive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
    }
}
