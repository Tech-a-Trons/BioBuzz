package org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

import dev.nextftc.robot.Mechanism;

/**
 * Field-centric mecanum drive with stick shaping and a hold-to-slow button.
 * Only writes a motor when its power actually changed.
 */
public class Drivetrain implements Mechanism {
    private static final double EPS = 0.005;

    // false = robot-centric (sticks move the robot relative to its own front), true = field-centric
    private static final boolean FIELD_CENTRIC = false;

    // ---- driver feel (tune) ----
    private static final double DEADBAND = 0.05;
    private static final double CURVE = 0.6;       // 0 = linear, 1 = fully cubic (finer control near center)
    private static final double SLOW_SCALE = 0.3;  // while the slow button (b) is held

    private DoubleSupplier headingSource = () -> 0;  // robot heading in radians, CCW positive (Pedro)
    private double forwardHeading = 0;               // robot heading that counts as "driver forward"
    private boolean backWasPressed = false;

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private double lfl = 2, lbl = 2, lfr = 2, lbr = 2; // impossible values force the first write

    public void init(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.dcMotor.get("fl");
        backLeft   = hardwareMap.dcMotor.get("bl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backRight  = hardwareMap.dcMotor.get("br");

        // Same directions as the simple TurretOp
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setHeadingSource(DoubleSupplier headingRadians) { this.headingSource = headingRadians; }

    /** The robot heading (radians) that should count as "forward" for the driver. */
    public void setForwardHeading(double radians) { this.forwardHeading = radians; }

    /** Makes the robot's current facing the driver's forward. Does NOT touch odometry. */
    public void resetForward() { this.forwardHeading = headingSource.getAsDouble(); }

    /**
     * Schedules the infinite field-centric drive command (call after init).
     * Left stick = translate, right stick x = turn, hold b = slow mode, back = re-zero forward.
     */
    public void startDrive(Gamepad gamepad) {
        infinite(() -> {
            if (gamepad.back && !backWasPressed) resetForward();
            backWasPressed = gamepad.back;

            double scale = gamepad.b ? SLOW_SCALE : 1.0;
            double fwd = shape(-gamepad.left_stick_y) * scale;
            double str = shape(gamepad.left_stick_x) * scale;
            double rot = shape(gamepad.right_stick_x) * scale;

            // Rotate the stick vector by -(heading - forwardHeading) to get robot-centric input
            double robotStr = str, robotFwd = fwd;
            if (FIELD_CENTRIC) {
                double h = headingSource.getAsDouble() - forwardHeading;
                double cos = Math.cos(h), sin = Math.sin(h);
                robotStr = str * cos + fwd * sin;
                robotFwd = -str * sin + fwd * cos;
            }

            drive(robotFwd, robotStr, rot);
        }).schedule();
    }

    // deadband (rescaled so output still starts at 0) + blend of linear and cubic
    private static double shape(double v) {
        double a = Math.abs(v);
        if (a < DEADBAND) return 0;
        a = (a - DEADBAND) / (1 - DEADBAND);
        double out = CURVE * a * a * a + (1 - CURVE) * a;
        return Math.copySign(out, v);
    }

    public void drive(double forward, double strafe, double turn) {
        double denom = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
        double fl = (forward + strafe + turn) / denom;
        double bl = (forward - strafe + turn) / denom;
        double fr = (forward - strafe - turn) / denom;
        double br = (forward + strafe - turn) / denom;

        if (changed(fl, lfl)) { frontLeft.setPower(fl);  lfl = fl; }
        if (changed(bl, lbl)) { backLeft.setPower(bl);   lbl = bl; }
        if (changed(fr, lfr)) { frontRight.setPower(fr); lfr = fr; }
        if (changed(br, lbr)) { backRight.setPower(br);  lbr = br; }
    }

    // always send a transition to/from zero so the motor never gets stuck at a tiny power
    private static boolean changed(double now, double last) {
        return Math.abs(now - last) > EPS || (now == 0 && last != 0);
    }
}