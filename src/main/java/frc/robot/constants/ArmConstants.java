package frc.robot.constants;

public class ArmConstants {
    public static final int kExtensionPort = 4;
    public static final int kPivotPort1 = 5;        
    public static final int kPivotPort2 = 6;
        
    public static final int kPivotEncoderSourceA = 1;
    public static final int kPivotEncoderSourceB = 2;
    public static final int kExtensionEncoderSourceA = 3;
    public static final int kExtensionEncoderSourceB = 4;

    public static final double kPivotEncoderDistance = 0; // TODO: Find calculation for encoder distance
    public static final double kExtensionEncoderDistance = 0; // TODO: distancePerPulse = pulley circum / pulses per rotation

    public static final int kPivotSwitchMinChannel = 5;
    public static final int kPivotSwitchMaxChannel = 6;
    public static final int kEncoderSwitchMinChannel = 7;
    public static final int kEncoderSwitchMaxChannel = 8;

    public static final double kMaxHeight = 0.0; // inches
    public static final double kArmRetractedLength = 0.0; // inches
    public static final double kArmMaxExtensionLength = 0.0; // inches
    public static final double kEndEffectorLength = 0.0; // inches
    public static final double kMaxPivotAngle = 0.0; // degrees
    public static final double kMinPivotAngle = 0.0; // degrees
}
