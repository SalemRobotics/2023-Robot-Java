package frc.robot.constants;

import org.opencv.core.Point;

public enum ArmPresets {
    //x for left (red) big goal is 25.624
    LOW_GOAL(0, 0),
    MID_GOAL(0, 0),
    HIGH_GOAL(0, 0);

    private double x, y;

    ArmPresets(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point get() {
        return new Point(x, y);
    }
}
