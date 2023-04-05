package frc.robot.constants;

public class DrivetrainConstants {
    public static final int kLeftFrontPort=1;
    public static final int kLeftTopPort=2;
    public static final int kLeftBackPort=3;

    public static final int kRightFrontPort=7;
    public static final int kRightBackPort=8;
    public static final int kRightTopPort=9; 

    public static final int kPidgeonPort=1;

    public static final double kPositionFactor = (0.39898) * 0.112; // meters per 1 revolution / cpr / gear ratio
    public static final double kVelocityFactor = kPositionFactor / 60.0;

    public static final double ksVolts=0.13422;
    public static final double kvVoltSecondsPerMeter=2.7125;
    public static final double kaVoltSecondsSquaredPerMeter=0.24729;
    public static final double trackWidthMeters=0.69215;
    public static final double kMaxSpeedMetersPerSecond=2.0;
    public static final double kMaxAccelerationMetersPerSecondSquared=2.0;
    public static final double kGyroAlignError=2.5;

    public static final double kRamseteB=2;
    public static final double kRamseteZeta=0.7;
    
    // too high oscilation, decrease P, then I, double check D
    public static final double kPDriveVel=5.0;
    public static final double kIDriveVel=5.0;
    public static final double KDDriveVel=0.085;
    
    public static final double kPCharger=0.006;
    public static final double kICharger=0;
    public static final double kDCharger=0.0005; // maybe turn down lower

    public static final double kPTag=0;
    public static final double kITag=0;
    public static final double kDTag=0;

    public static final String kCharger = "Charger";
    public static final String kChargerMobilityPath = "Charger Mobility";
    public static final String k2ScoreCube = "2 Score Cube";
    public static final String k2ScoreCubeEngage = "2 Score Cube And Engage";
    public static final String k25ScoreCube = "2.5 Score Cube";
    public static final String k25ScoreCubeEngage = "2.5 Score Cube Engage";
    public static final String k3ScoreCube = "3 Score Cube";
    public static final String k3ScoreCubeEngage = "3 Score Cube and Engage";
    public static final String k1ScoreLow = "1 Score Low(Usable Anywhere)";
    public static final String k1ScoreHigh = "1 Score High(Usable Anywhere)";
    public static final String kBump1ScoreMobility = "Bump 1 Score + Mobility";
    public static final String kBump2Score = "Bump 2 Score";
    public static final String kBump3Score = "Bump 3 Score (do not use)";
    public static final String kCharger1ScoreMobility = "Charger 1 Score + Mobility";
    public static final String kFlat1ScoreMobility = "Flat Side 1 Score + Mobility";
}
