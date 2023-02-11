package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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

    SparkMaxPIDController leftController = leftFrontMotor.getPIDController();
    SparkMaxPIDController righController = rightFrontMotor.getPIDController();

    RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
    RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();

    DifferentialDrive drive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

    PigeonIMU gyro = new PigeonIMU(DrivetrainConstants.kPidgeonPort);

    // Create gyro and odometry
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
        getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    double kP=0, kI=0;

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
        odometry.update(getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
        updateGyroShuffleboard();
        updatePID();
    }

    void updateGyroShuffleboard() {
        /** Gyro **/
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        // pitch and roll are swapped
        SmartDashboard.putNumber("Pitch", gyro.getRoll()); 
        SmartDashboard.putNumber("Roll", gyro.getPitch());
    }

    void updatePID() {
        kP = SmartDashboard.getNumber("P", kP);
        kI = SmartDashboard.getNumber("I", kI);
    }
    
    public CommandBase alignToCharger() {
        return run(
            () -> {
                new PIDCommand(
                    new PIDController(kP, kI, 0), 
                    gyro::getRoll, 
                    0, 
                    output -> {
                        leftFrontMotor.set(output);
                        rightFrontMotor.set(output);
                    }, 
                    this
                );
            }
        );
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
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }
    
    public double getAverageEncoderDistance() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }
    
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(
            getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
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
