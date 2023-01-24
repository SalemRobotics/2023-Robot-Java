package frc.robot.constants;

public class ArmConstants {
    public static final int kPivotPort1 = 6;        
    public static final int kPivotPort2 = 7;
    public static final int kExtensionPort = 8;
        
    public static final int kPivotEncoderPort = 0;
    public static final int kExtensionEncoderSourceA = 0;
    public static final int kExtensionEncoderSourceB = 0;

    public static final int kPivotEncoderFrequency = 976;
    public static final double kPivotEncoderPulseMin = 1/1025; // microseconds
    public static final double kPivotEncoderPulseMax = 1024/1025; // microseconds
    public static final double kExtensionEncoderDistance = 0; // TODO: distancePerPulse = pulley circum / pulses per rotation

    public static final double kMaxHeight = 0.0; // inches
    public static final double kArmRetractedLength = 0.0; // inches
    public static final double kArmMaxExtensionLength = 0.0; // inches
    public static final double kEndEffectorLength = 0.0; // inches
    public static final double kMaxPivotAngle = 0.0; // degrees
    public static final double kMinPivotAngle = 0.0; // degrees
}
