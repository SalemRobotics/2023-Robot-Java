package frc.robot.constants;

import org.opencv.core.Point;

/**
 * An enum containing the preset positions for the {@link frc.robot.subsystems.Arm} subsystem.
 */
public enum ArmPresets {
    //x for left (red) hybrid goal is 25.624
    //y for left (red) hybrid goal isn't really relevant
    //x for all regular hybrid goals is 23.9525
    //y for all hybrid goals is kind of 0
    //x for all regular low shelf goals is 39.792
    //y for all regular low shelf goals is 20.563
    //x for all regular high shelf goals is 57.259
    //y for all regular high shelf goals is 32.533

    DEFAULT(0, 0),
    INTAKE(0, 0),
    LOW_GOAL(0, 0),
    MID_GOAL(0, 0),
    HIGH_GOAL(0, 0);

    private double x, y;

    ArmPresets(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public final Point value = new Point(x, y);
}
