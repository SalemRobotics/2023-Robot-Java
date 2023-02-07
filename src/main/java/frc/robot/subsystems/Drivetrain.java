package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivetrainConstants;

/**
 * Subsystem class for the drivetrain subsystem.
 */
public class Drivetrain extends SubsystemBase {
    CANSparkMax leftFrontMotor = new CANSparkMax(DrivetrainConstants.kLeftFrontPort, MotorType.kBrushless);
    CANSparkMax leftBackMotor = new CANSparkMax(DrivetrainConstants.kLeftBackPort, MotorType.kBrushless);
    CANSparkMax leftTopMotor = new CANSparkMax(DrivetrainConstants.kLeftTopPort, MotorType.kBrushless);

    CANSparkMax rightFrontMotor = new CANSparkMax(DrivetrainConstants.kRightFrontPort, MotorType.kBrushless);
    CANSparkMax rightBackMotor = new CANSparkMax(DrivetrainConstants.kRightBackPort, MotorType.kBrushless);
    CANSparkMax rightTopMotor = new CANSparkMax(DrivetrainConstants.kRightTopPort, MotorType.kBrushless);

    DifferentialDrive drive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

    PigeonIMU gyro = new PigeonIMU(DrivetrainConstants.kPidgeonPort);

    // Create gyro and odometry
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
        getRotation2d(), leftFrontMotor.getEncoder().getPosition(), rightFrontMotor.getEncoder().getPosition());

    public Drivetrain() {
        leftTopMotor.follow(leftFrontMotor);
        leftBackMotor.follow(leftFrontMotor);

        rightFrontMotor.setInverted(true);
        rightTopMotor.follow(rightFrontMotor);
        rightBackMotor.follow(rightFrontMotor);

        zeroHeading();
        resetEncoders();
    }

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), leftFrontMotor.getEncoder().getPosition(), rightFrontMotor.getEncoder().getPosition());
        updateShuffleboard();
    }

    void updateShuffleboard() {
        /** Gyro **/
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        // pitch and roll are swapped
        SmartDashboard.putNumber("Pitch", gyro.getRoll()); 
        SmartDashboard.putNumber("Roll", gyro.getPitch());
    }

    /** 
     * sets arcade drive for motors
     * @param fwd Forward axis of drivetrain (Y axis)
     * @param rot Axis of rotation of drivetrain (X axis) 
     */
    public CommandBase arcadeDrive(DoubleSupplier forward, DoubleSupplier rotation) {
        return run(
            () -> {
                drive.arcadeDrive(forward.getAsDouble(), -rotation.getAsDouble(), true);
            }
        );
    }
 
    /** 
     * sets volts to control motors at a set speed 
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        rightFrontMotor.set(rightVolts);
        leftFrontMotor.set(leftVolts);
        drive.feed();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);     
    }

    public Rotation2d getRotation2d() {
        double yaw = gyro.getYaw();
        return Rotation2d.fromDegrees(Math.IEEEremainder(yaw, 360.0d));
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftFrontMotor.getEncoder().getVelocity(), rightFrontMotor.getEncoder().getVelocity());
    }
    
    public double getAverageEncoderDistance() {
        return (leftFrontMotor.getEncoder().getPosition() + rightFrontMotor.getEncoder().getPosition()) / 2.0;
    }
    
    public void resetEncoders() {
        leftFrontMotor.getEncoder().setPosition(0);
        rightFrontMotor.getEncoder().setPosition(0);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(
            getRotation2d(), leftFrontMotor.getEncoder().getPosition(), rightFrontMotor.getEncoder().getPosition(), pose);
    }
    /*
     * uses the rotation to find how many degrees the bot is turned
     */
    public void getHeading() {
        getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        double[] xyz = new double[3];
        gyro.getRawGyro(xyz);
        return xyz[1];
    }
    /*
     * sets the yaw to 0.0
     */
    public void zeroHeading() {
        gyro.setYaw(0.0);
    }  
}
