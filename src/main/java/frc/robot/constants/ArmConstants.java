package frc.robot.constants;

public class ArmConstants {
    public static final int kPivotPort1 = 6;        
    public static final int kPivotPort2 = 7;
    public static final int kExtensionPort = 8;
        
    public static final int kPivotEncoderSourceA = 0;
    public static final int kPivotEncoderSourceB = 0;
    public static final int kExtensionEncoderSourceA = 0;
    public static final int kExtensionEncoderSourceB = 0;

    public static final double kPivotEncoderDistance = 0; // TODO: Find calculation for encoder distance
    public static final double kExtensionEncoderDistance = 0; // TODO: distancePerPulse = pulley circum / pulses per rotation

    public static final int kPivotSwitchMinChannel = 0;
    public static final int kPivotSwitchMaxChannel = 0;
    public static final int kEncoderSwitchMinChannel = 0;
    public static final int kEncoderSwitchMaxChannel = 0;

    public static final double kMaxHeight = 0.0; // inches
    public static final double kArmRetractedLength = 0.0; // inches
    public static final double kArmMaxExtensionLength = 0.0; // inches
    public static final double kEndEffectorLength = 0.0; // inches
    public static final double kMaxPivotAngle = 0.0; // degrees
    public static final double kMinPivotAngle = 0.0; // degrees
}
