package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Season.Auto.pedro.Constants;

import java.util.Objects;

public class OdoPID {

    //Constants from Niranjan's tele
    // ---- Shooter PID constants (tune these) ----
    private static final double kS = 0.12;     // power to overcome friction
    private static final double kV = 0.0074;   // power per RPM
    private static final double kP = 0.02;     // error is in deg/s
    private static final double kI = 0.0;      // start at 0, add only if it settles below target
    private static final double kD = 0.0;      // start at 0, add only if it oscillates

    private static double TARGET_RPM = 27;
    private static final double DEG_PER_SEC_PER_RPM = 6.0; // RPM x 6 = deg/s
    private static final double MAX_INTEGRAL = 0.3;        // anti-windup limit (in power units)

    Follower follower;

    double integral = 0;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    DcMotorEx outtakeLeft;

    public void init(HardwareMap hardwareMap) {
        DcMotorEx outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeleft");
        outtakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        follower = Constants.create(hardwareMap);
    }

    public void reset() {
        integral = 0;
        lastError = 0;

        outtakeLeft.setPower(0);
    }

    public void run() {
        double dt = timer.seconds();
        timer.reset();

        double currentDegPerSec = outtakeLeft.getVelocity(AngleUnit.DEGREES);
        double currentRpm = currentDegPerSec / DEG_PER_SEC_PER_RPM;
        double shooterPower = 0;

        double targetDegPerSec = TARGET_RPM * DEG_PER_SEC_PER_RPM;
        double error = targetDegPerSec - currentDegPerSec;

        // Integral with anti-windup
        if (dt > 0) {
            integral += error * dt * kI;
            integral = Range.clip(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
        }

        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        double feedforward = kS + kV * TARGET_RPM;
        shooterPower = feedforward + kP * error + integral + kD * derivative;
        shooterPower = Range.clip(shooterPower, 0, 1);

        outtakeLeft.setPower(shooterPower);
    }

    public String alliance;

    //Later, this will get input from the auto for Alliance
    public void Alliance() {
        alliance = "RED";
        if (gamepad1.dpad_left) {
            alliance = "BLUE";
        }

    }

    public void OdoRun() {

        follower.update();               // must run every loop so the pose stays current
        Pose robotPose = follower.pose();

        double x = robotPose.x();              // inches
        double y = robotPose.y();              // inches

        double xt = 0;
        double yt = 0;

        if (Objects.equals(alliance, "BLUE")) {
            if (y >= 70.75) {
                xt = 84;
                yt = 82;
            } else if (y < 70.75) {
                xt = 84;
                yt = 56;
            }
        } else if (Objects.equals(alliance, "RED")) {
            if (y >= 70.75) {
                xt = 58;
                yt = 82;
            } else if (y < 70.75) {
                xt = 58;
                yt = 56;
            }
        }

//        if (x >= 70.75) {
//            if (y >= 70.75) {
//                xt = 84;
//                yt = 82;
//            } else if (y < 70.75) {
//                xt = 84;
//                yt = 56;
//            }
//        } else if (x < 70.75) {
//            if (y >= 70.75) {
//                xt = 58;
//                yt = 82;
//            } else if (y < 70.75) {
//                xt = 58;
//                yt = 56;
//            }
//        }

        double xdiff = xt - x;
        double ydiff = yt - y;

        double dist = Math.sqrt(Math.pow(xdiff,2)+Math.pow(ydiff,2));

        TARGET_RPM = 0.84375*(dist)+0;

        double dt = timer.seconds();
        timer.reset();

        double currentDegPerSec = outtakeLeft.getVelocity(AngleUnit.DEGREES);
        double currentRpm = currentDegPerSec / DEG_PER_SEC_PER_RPM;
        double shooterPower = 0;

        double targetDegPerSec = TARGET_RPM * DEG_PER_SEC_PER_RPM;
        double error = targetDegPerSec - currentDegPerSec;

        // Integral with anti-windup
        if (dt > 0) {
            integral += error * dt * kI;
            integral = Range.clip(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
        }

        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        double feedforward = kS + kV * TARGET_RPM;
        shooterPower = feedforward + kP * error + integral + kD * derivative;
        shooterPower = Range.clip(shooterPower, 0, 1);

        outtakeLeft.setPower(shooterPower);

        telemetry.addData("Robot X", x);
        telemetry.addData("Robot Y", y);
        telemetry.addData("Robot XT", xt);
        telemetry.addData("Robot YT", yt);
        telemetry.addData("Dist to goal", dist);
        telemetry.update();
    }

}