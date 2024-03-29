package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
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

    RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
    RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();

    DifferentialDrive drive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

    PigeonIMU gyro = new PigeonIMU(DrivetrainConstants.kPidgeonPort);

    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
        getRotation2d(), -rightEncoder.getPosition(), -leftEncoder.getPosition());

    double yaw, pitch;

    public Drivetrain() {
        leftTopMotor.follow(leftFrontMotor);
        leftBackMotor.follow(leftFrontMotor);

        rightFrontMotor.setInverted(true);
        rightTopMotor.follow(rightFrontMotor);
        rightBackMotor.follow(rightFrontMotor);

        leftEncoder.setPositionConversionFactor(DrivetrainConstants.kPositionFactor);
        rightEncoder.setPositionConversionFactor(DrivetrainConstants.kPositionFactor);

        leftEncoder.setVelocityConversionFactor(DrivetrainConstants.kVelocityFactor);
        rightEncoder.setVelocityConversionFactor(DrivetrainConstants.kVelocityFactor);

        zeroHeading();
        resetEncoders();
    }

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), -rightEncoder.getPosition(), -leftEncoder.getPosition());
        updateShuffleboard();

        yaw = -gyro.getYaw();
        pitch = gyro.getRoll() + DrivetrainConstants.kGyroAlignError;
    }

    /**
     * Updates Shuffleboard entries.
     */
    void updateShuffleboard() {
        /** Gyro **/
        SmartDashboard.putNumber("Yaw", yaw);
        // pitch and roll are swapped
        SmartDashboard.putNumber("Pitch", pitch); 
        SmartDashboard.putNumber("Roll", gyro.getPitch());

        // Left side motor current
        SmartDashboard.putNumber("LeftFrontMotor Current", leftFrontMotor.getOutputCurrent());
        SmartDashboard.putNumber("LeftTopMotor Current", leftTopMotor.getOutputCurrent());
        SmartDashboard.putNumber("LeftBackMotor Current", leftBackMotor.getOutputCurrent());

        // Right side motor current
        SmartDashboard.putNumber("RightFrontMotor Current", rightFrontMotor.getOutputCurrent());
        SmartDashboard.putNumber("RightTopMotor Current", rightTopMotor.getOutputCurrent());
        SmartDashboard.putNumber("RightBackMotor Current", rightBackMotor.getOutputCurrent());
    }
    
    /**
     * A command to drive the robot in whichever direction it needs to 
     * in order to maintain a pitch of 0. This Is used to make sure the robot stays docked on the charger. 
     * @return Runnable command.
     */
    public Command alignToCharger() {
        return new ParallelRaceGroup(
            new PIDCommand(
                new PIDController(
                    DrivetrainConstants.kPCharger, DrivetrainConstants.kICharger, DrivetrainConstants.kDCharger
                ), 
                () -> { return pitch; }, 
                0, 
                output -> {
                    leftFrontMotor.set(output);
                    rightFrontMotor.set(output);
                }, 
                this
            ),
            new WaitCommand(5)
        );
    }

    /**
     * A command to drive the robot over the charger and back using the gyroscope. 
     * @return Runnable command.
     */
    public Command taxiOverCharger() { // Needs cleanup
        return new SequentialCommandGroup(
            run(
                () -> {
                    drive.arcadeDrive(0.6, 0);
                }
            ).until( () -> { return pitch > 8.0; } ),
            run(
                () -> {
                    drive.arcadeDrive(0.5, 0);
                }
            ).until( () -> { return Math.abs(pitch) < 3.0; } ),
            new ParallelRaceGroup(
                run(
                    () -> {
                        drive.arcadeDrive(0.5, 0);
                    }
                ),
                new WaitCommand(0.75)
            ),
            run(
                () -> {
                    drive.arcadeDrive(-0.5, 0);
                    
                }
            ).until( () -> { return pitch > 8.0; } ),
            run(
                () -> {
                    drive.arcadeDrive(-0.5, 0);
                }
            ).until( () -> { return Math.abs(pitch) < 6.0; } ),
            alignToCharger()
        );
    }

    public Command alignToApriltag() {
        double targetAngle = LimelightHelpers.getPythonScriptData("limelight")[1];
        return new PIDCommand(
            new PIDController(
                DrivetrainConstants.kPTag, DrivetrainConstants.kITag, DrivetrainConstants.kDTag
            ), 
            gyro::getYaw, 
            targetAngle, 
            output -> {
                drive.arcadeDrive(0, -output);
            }, 
            this
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
        rightFrontMotor.setVoltage(-leftVolts);
        leftFrontMotor.setVoltage(-rightVolts);
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
        return Rotation2d.fromDegrees(Math.IEEEremainder(yaw, 360.0d));
    }

    /**
     * Gets the velocity of the left and right sides of the drivetrain. 
     * @return A {@linkplain DifferentialDriveWheelSpeeds} object. 
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(-rightEncoder.getVelocity(), -leftEncoder.getVelocity());
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
            getRotation2d(), -rightEncoder.getPosition(), -leftEncoder.getPosition(), pose);
    }
    /*
     * Pigeon gyro rotation in degrees.
     */
    public double getHeading() {
        return getRotation2d().getDegrees();
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
