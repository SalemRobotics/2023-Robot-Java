package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
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
 * Allows tank drive control by a player-operated xbox controller.
 * Contains methods for autonomous control, including external devices such as a gyroscope.
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

    /**
     * Updates Shuffleboard entries.
     */
    void updateGyroShuffleboard() {
        /** Gyro **/
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        // pitch and roll are swapped
        SmartDashboard.putNumber("Pitch", gyro.getRoll()); 
        SmartDashboard.putNumber("Roll", gyro.getPitch());
    }

    /**
     * Sends PID constants to Shuffleboard as editable fields. 
     */
    void updatePID() {
        kP = SmartDashboard.getNumber("P", kP);
        kI = SmartDashboard.getNumber("I", kI);
    }
    
    /**
     * A command to drive the robot in whichever direction it needs to 
     * in order to maintain a pitch of 0. This Is used to make sure the robot stays docked on the charger. 
     * @return Runnable command.
     */
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
     * Sets arcade drive for both sides of the drivetrain.
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
     * Sets the amount voltage for each side of the drivetrain. 
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        rightFrontMotor.setVoltage(rightVolts);
        leftFrontMotor.setVoltage(leftVolts);
        drive.feed();
    }

    /**
     * Get odometry position on the field
     * @return Position (x, y), in meters
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Configure the scaling factor for using drive methods with motor controllers in a mode other than 
     * PercentVbus or to limit the maximum output.
     * The default value is 1.0.
     * @param maxOutput Multiplied with the output percentage computed by the drive functions.
     */
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);     
    }

    /**
     * Gets the current rotation of the robot.
     * @return A {@linkplain Rotation2d} representing the rotation of the robot.
     */
    public Rotation2d getRotation2d() {
        double yaw = gyro.getYaw();
        return Rotation2d.fromDegrees(Math.IEEEremainder(yaw, 360.0d));
    }

    /**
     * Gets the velocity of the left and right sides of the drivetrain. 
     * @return A {@linkplain DifferentialDriveWheelSpeeds} object. 
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }
    
    /**
     * Gets the average of the distance travled by the left and right encoders
     * @return Average distance, in meters
     */
    public double getAverageEncoderDistance() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }
    
    /**
     * Resets encoders to 0.
     */
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    /**
     * Resets the position of the robot on the field to (0,0) relative to it's current position
     * @param pose Current position on the field
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(
            getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
    }
    /*
     * Pigeon gyro rotation in degrees.
     */
    public void getHeading() {
        getRotation2d().getDegrees();
    }

    /**
     * Gets the turn rate of the pigeon.
     * @return
     */
    public double getTurnRate() {
        double[] xyz = new double[3];
        gyro.getRawGyro(xyz);
        return xyz[1];
    }
    /*
     * Sets the yaw to 0.0
     */
    public void zeroHeading() {
        gyro.setYaw(0.0);
    }  
}
