package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems.comp1;

/**
 * Picks which of the two same-alliance goals to aim at, using Pedro field coords (0..141.5).
 * Aims at the goal on the robot's side of the field; switches to the other goal after the
 * robot crosses the midline (with a small dead zone so it doesn't flicker on the line).
 * The turret AND the shooter both read from this, so they always agree on the target.
 */
public class GoalSelector {
    // Your tuned goal points
    private static final double RED_X = 58, BLUE_X = 84;
    private static final double UPPER_Y = 82, LOWER_Y = 56;

    private static final double MID_Y = 70.75;
    private static final double HYSTERESIS = 3.0; // inches past the midline before switching

    private boolean red = true;
    private boolean upper = true;
    private boolean initialized = false;

    public void setAlliance(String alliance) {
        red = !"BLUE".equals(alliance);
        initialized = false;
    }

    /** Safe to call from several places per loop: same input gives the same result. */
    public void update(double robotY) {
        if (!initialized) {
            upper = robotY >= MID_Y;
            initialized = true;
        } else if (upper && robotY < MID_Y - HYSTERESIS) {
            upper = false;
        } else if (!upper && robotY > MID_Y + HYSTERESIS) {
            upper = true;
        }
    }

    public double x() { return red ? RED_X : BLUE_X; }
    public double y() { return upper ? UPPER_Y : LOWER_Y; }
    public boolean isUpper() { return upper; }
}