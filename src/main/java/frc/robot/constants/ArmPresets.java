package frc.robot.constants;

import org.opencv.core.Point;

/**
 * An enum containing the preset positions for the {@link frc.robot.subsystems.Arm} subsystem.
 */
public enum ArmPresets {
    DEFAULT(-11.4, -0.5),

    CUBE_LOW_GOAL(0, -1.5),
    CUBE_MID_GOAL(-7, -9.67),
    CUBE_HIGH_GOAL(-7.55, -36),

    CONE_LOW_GOAL(-1.6, -10),
    CONE_MID_GOAL(-7.75, -9.67),
    CONE_HIGH_GOAL(-7.75, -38);

    public final Point value;

    ArmPresets(double x, double y) {
        value = new Point(x, y);
    }
}
