package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem class for the drivetrain subsystem
 */
public class Drivetrain extends SubsystemBase {
    CANSparkMax leftFrontMotor = new CANSparkMax(Constants.DrivetrainConstants.kLeftFrontPort, MotorType.kBrushless);
    CANSparkMax leftBackMotor = new CANSparkMax(Constants.DrivetrainConstants.kLeftBackPort, MotorType.kBrushless);
    CANSparkMax leftTopMotor = new CANSparkMax(Constants.DrivetrainConstants.kLeftTopPort, MotorType.kBrushless);

    CANSparkMax rightFrontMotor = new CANSparkMax(Constants.DrivetrainConstants.kRightFrontPort, MotorType.kBrushless);
    CANSparkMax rightBackMotor = new CANSparkMax(Constants.DrivetrainConstants.kRightBackPort, MotorType.kBrushless);
    CANSparkMax rightTopMotor = new CANSparkMax(Constants.DrivetrainConstants.kRightTopPort, MotorType.kBrushless);

    DifferentialDrive drive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

    PigeonIMU gyro = new PigeonIMU(1);

    // Create gyro and odometry
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
        getRotation2d(), leftFrontMotor.getEncoder().getPosition(), rightFrontMotor.getEncoder().getPosition());

    public Drivetrain() {
        leftBackMotor.follow(leftFrontMotor);
        leftTopMotor.follow(leftFrontMotor);

        rightFrontMotor.setInverted(true);
        rightBackMotor.follow(rightFrontMotor);
        rightTopMotor.follow(rightFrontMotor);

        zeroHeading();
        resetEncoders();
    }

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), leftFrontMotor.getEncoder().getPosition(), rightFrontMotor.getEncoder().getPosition());
    }
 
    /** 
     * sets volts to control motors at a set speed 
     */
    public void tankDriveVolts(Double speed) {
        rightFrontMotor.set(speed);
        leftFrontMotor.set(speed);
        drive.feed();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    /** 
     * sets arcade drive for motors
     * fwd is Forward axis of drivetrain (Y axis)
     * rot is Axis of rotation of drivetrain (X axis) 
     */
    public void arcadeDrive(double fwd, double rot) {
        drive.arcadeDrive(-fwd, rot, true);
    }

    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);     
    }

    public Rotation2d getRotation2d() {
        double yaw = gyro.getYaw();
        //SmartDashboard.putNumber("Yaw", yaw);
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

    public void getHeading() {
        getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        double[] xyz = new double[3];
        gyro.getRawGyro(xyz);
        return xyz[1];
    }

    public void zeroHeading() {
        gyro.setYaw(0.0);
    }  
}
