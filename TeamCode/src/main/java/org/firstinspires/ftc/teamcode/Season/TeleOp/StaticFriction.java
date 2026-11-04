package org.firstinspires.ftc.teamcode.Season.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StaticFriction extends LinearOpMode {
    private static final double kS = 0.12;     // power to overcome friction
    private static final double kV = 0.001;    // power per RPM
    private static final double kP = 0.027;    // error is in deg/s
    private static final double kI = 0.0;
    private static final double kD = 0.03;

    private static final double TARGET_RPM = 29;
    private static final double DEG_PER_SEC_PER_RPM = 6.0; // RPM x 6 = deg/s
    private static final double MAX_INTEGRAL = 0.3;        // anti-windup limit (in power units)
    private static final double SHOOTER_SIGN = -1;
    @Override
    public void runOpMode() throws InterruptedException {

    }
}
