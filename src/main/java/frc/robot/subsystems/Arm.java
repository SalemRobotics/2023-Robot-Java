package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    CANSparkMax pivotMotor1 = new CANSparkMax(Constants.ArmConstants.kPivotPort1, MotorType.kBrushless);
    CANSparkMax pivotMotor2 = new CANSparkMax(Constants.ArmConstants.kPivotPort2, MotorType.kBrushless);

    CANSparkMax extensionMotor = new CANSparkMax(Constants.ArmConstants.kExtensionPort, MotorType.kBrushless);

    AnalogEncoder pivotEncoder = new AnalogEncoder(Constants.ArmConstants.kPivotEncoderPort);
    Encoder extensionEncoder = new Encoder(Constants.ArmConstants.kExtensionEncoderSourceA, Constants.ArmConstants.kExtensionEncoderSourceB);

    SparkMaxPIDController pivotController;
    SparkMaxPIDController extensionController;

    double kPivotP, kPivotI, kPivotD,
        kPivotIz, kPivotFF, kPivotMaxOutput, kPivotMinOutput,
        kPivotMaxVel, kPivotMinVel, kPivotMaxAcc, kPivotAllowedErr;

    double kExtP, kExtI, kExtD, 
        kExtIz, kExtFF, kExtMaxOutput, kExtMinOutput,
        kExtMaxVel, kExtMinVel, kExtMaxAcc, kExtAllowedErr;

    public Arm() {
        pivotMotor2.follow(pivotMotor1);

        extensionEncoder.setDistancePerPulse(0); // TODO: distancePerPulse = pulley circum / pulses per rotation

        // Pivot Loop Consts 
        // TODO: Profile arm to get the below values. Currently default.
        kPivotP = 0.1; 
        kPivotI = 1e-4;
        kPivotD = 1; 
        kPivotIz = 0; 
        kPivotFF = 0; 
        kPivotMaxOutput = 1; 
        kPivotMinOutput = -1;

        pivotController = pivotMotor1.getPIDController();
        pivotController.setP(kPivotP);
        pivotController.setI(kPivotI);
        pivotController.setD(kPivotD);
        pivotController.setIZone(kPivotIz);
        pivotController.setFF(kPivotFF);
        pivotController.setOutputRange(kPivotMinOutput, kPivotMaxOutput);

        // Extension Loop Consts
        // TODO: Profile arm to get the below values. Currently default.
        kExtP = 0.1; 
        kExtI = 1e-4;
        kExtD = 1; 
        kExtIz = 0; 
        kExtFF = 0; 
        kExtMaxOutput = 1; 
        kExtMinOutput = -1;

        extensionController = extensionMotor.getPIDController();
        extensionController.setP(kExtP);
        extensionController.setI(kExtI);
        extensionController.setD(kExtD);
        extensionController.setIZone(kExtIz);
        extensionController.setFF(kExtFF);
        extensionController.setOutputRange(kExtMinOutput, kExtMaxOutput);
    }

    public void setPosition(double x, double y) {
        double angle = Math.atan(y/x);
        double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        pivotController.setReference(angle, ControlType.kSmartMotion);
        extensionController.setReference(distance, ControlType.kSmartMotion);
    }

    @Override
    public void periodic() {
        updatePivotConsts();
        updateExtensionConsts();
    }

    /**
     * TODO: once satisfied with constants, read them from a file instead of smart dashboard
     */
    void updatePivotConsts() {
        double p =SmartDashboard.getNumber("Error Multiplier (P)", kPivotP);
        double i =SmartDashboard.getNumber("Sum Error (I)", kPivotI);
        double d =SmartDashboard.getNumber("Slope Error (D)", kPivotD);
        double iz =SmartDashboard.getNumber("Integral Effective Error Range (IZone)", kPivotIz);
        double ff =SmartDashboard.getNumber("Control Loop Gain (Feed Forward)", kPivotFF);
        double min =SmartDashboard.getNumber("Min Output", kPivotMinOutput);
        double max =SmartDashboard.getNumber("Max Output", kPivotMaxOutput);
        double minV = SmartDashboard.getNumber("Min Velocity", kPivotMinVel);
        double maxV = SmartDashboard.getNumber("Max Velocity", kPivotMaxVel);
        double maxA = SmartDashboard.getNumber("Max Acceleration", kPivotMaxAcc);
        double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", kPivotAllowedErr); 

        if (p != kPivotP) { pivotController.setP(p); kPivotP = p; }
        if (i != kPivotI) { pivotController.setI(i); kPivotI = i; }
        if (d != kPivotD) { pivotController.setD(d); kPivotD = d; }
        if (iz != kPivotIz) { pivotController.setIZone(iz); kPivotIz = iz; }
        if (ff != kPivotFF) { pivotController.setFF(p); kPivotFF = ff; }
        if ((max != kPivotMaxOutput) || (min != kPivotMinOutput)) {
            pivotController.setOutputRange(min, max);
            kPivotMinOutput = min; kPivotMaxOutput = max;
        }
        if (maxV != kPivotMaxVel) { pivotController.setSmartMotionMaxVelocity(maxV, 0); kPivotMaxVel = maxV; }
        if (minV != kPivotMinVel) { pivotController.setSmartMotionMinOutputVelocity(minV, 0); kPivotMinVel = minV; }
        if (maxA != kPivotMaxAcc) { pivotController.setSmartMotionMaxAccel(maxA, 0); kPivotMaxAcc = maxA; }
        if (allE != kPivotAllowedErr) { pivotController.setSmartMotionAllowedClosedLoopError(allE, 0); kPivotAllowedErr = allE; }
    }

    /**
     * TODO: once satisfied with constants, read them from a file instead of smart dashboard
     */
    void updateExtensionConsts() {
        double p =SmartDashboard.getNumber("Error Multiplier (P)", kExtP);
        double i =SmartDashboard.getNumber("Sum Error (I)", kExtI);
        double d =SmartDashboard.getNumber("Slope Error (D)", kExtD);
        double iz =SmartDashboard.getNumber("Integral Effective Error Range (IZone)", kExtIz);
        double ff =SmartDashboard.getNumber("Control Loop Gain (Feed Forward)", kExtFF);
        double min =SmartDashboard.getNumber("Min Output", kExtMinOutput);
        double max =SmartDashboard.getNumber("Max Output", kExtMaxOutput);
        double minV = SmartDashboard.getNumber("Min Velocity", kExtMinVel);
        double maxV = SmartDashboard.getNumber("Max Velocity", kExtMaxVel);
        double maxA = SmartDashboard.getNumber("Max Acceleration", kExtMaxAcc);
        double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", kExtAllowedErr); 

        if (p != kExtP) { extensionController.setP(p); kExtP = p; }
        if (i != kExtI) { extensionController.setI(i); kExtI = i; }
        if (d != kExtD) { extensionController.setD(d); kExtD = d; }
        if (iz != kExtIz) { extensionController.setIZone(iz); kExtIz = iz; }
        if (ff != kExtFF) { extensionController.setFF(p); kExtFF = ff; }
        if ((max != kExtMaxOutput) || (min != kExtMinOutput)) {
            extensionController.setOutputRange(min, max);
            kExtMinOutput = min; kExtMaxOutput = max;
        }
        if (maxV != kExtMaxVel) { extensionController.setSmartMotionMaxVelocity(maxV, 0); kExtMaxVel = maxV; }
        if (minV != kExtMinVel) { extensionController.setSmartMotionMinOutputVelocity(minV, 0); kExtMinVel = minV; }
        if (maxA != kExtMaxAcc) { extensionController.setSmartMotionMaxAccel(maxA, 0); kExtMaxAcc = maxA; }
        if (allE != kExtAllowedErr) { extensionController.setSmartMotionAllowedClosedLoopError(allE, 0); kExtAllowedErr = allE; }
    }
}
