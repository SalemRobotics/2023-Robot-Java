package frc.robot.constants;

public class ArmConstants {
    public static final int kExtensionPort = 4;
    public static final int kPivotPort1 = 5;
    public static final int kPivotPort2 = 6;

    public static final double kPivotAngleFactor = 0; // TODO: 360 degrees / pulses per rotation
    public static final double kExtensionDistanceFactor = 0; // TODO: distancePerPulse = pulley circum / pulses per rotation
    
    public static final int kPivotSwitchMinChannel = 5;
    public static final int kPivotSwitchMaxChannel = 6;
    public static final int kExtensionSwitchMinChannel = 7;
    public static final int kExtensionSwitchMaxChannel = 8;

    public static final double kPPivot = 0.15;
    public static final double kPivotMaxSpeed = 0.18;

    public static final double kPExtension = 0.11;
    public static final double kExtensionMaxSpeed = 0.6;
    
    public static final double kMaxHeight = 77.5; // inches
    public static final double kArmRetractedLength = 42.75; // inches
    public static final double kArmMaxExtensionLength = 71.55; // inches
    public static final double kEndEffectorLength = 2.0; // inches
    public static final double kPivotAngleOffset = 132.68; // degrees
    public static final double kMaxPivotAngle = 0.0; // degrees
    public static final double kMinPivotAngle = 0.0; // degrees
    public static final double kMaxPivotVelocity = 0.0; // degrees/second
    public static final double kMaxPivotAccel = 0.0; // degrees/second^2
}
