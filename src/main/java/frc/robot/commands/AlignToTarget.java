package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.Drivetrain;

public class AlignToTarget extends CommandBase {
    
    final Drivetrain drivetrain;
    final LimelightResults llresults = new LimelightResults();

    public AlignToTarget(Drivetrain subsystem) {
        drivetrain = subsystem;
        addRequirements(drivetrain);
    }

    /**
     * TODO: calculate angle to target
     * @return
     */
    double getAngleToTarget() {
        double distance = LimelightHelpers.getPythonScriptData("limelight")[0];
        Pose2d botpose = llresults.targetingResults.getBotPose2d();
        return 0;
    }

    /**
     * TODO: Rotate robot to target angle
     */
    @Override
    public void execute() {
        
        
    }
}
