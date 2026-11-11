package org.firstinspires.ftc.teamcode.Season.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Season.Auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1.EasyTurret;
import org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1.GoalSelector;

@TeleOp(name = "TurretOp")
public class TurretOp extends LinearOpMode {

    // ---- Shooter PID constants (tune these) ----
    private static final double kS = 0;     // power to overcome friction
    private static final double kV = 0.023;    // power per RPM
    private static final double kP = 0.05;    // error is in deg/s
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private static final double TARGET_RPM = 27.8;
    private static final double DEG_PER_SEC_PER_RPM = 6.0; // RPM x 6 = deg/s
    private static final double MAX_INTEGRAL = 0.3;        // anti-windup limit (in power units)
    private static final double SHOOTER_SIGN = -1;

    // ---- Turret / localization setup ----
    private static final String ALLIANCE = "BLUE"; // change to "RED" when needed
    private static final Pose START_POSE = new Pose(77, 8.6, Math.toRadians(90));

    // Time to wait after opening the gate before feeding (was a blocking sleep(250))
    private static final double GATE_FEED_DELAY_SEC = 0.25;

    EasyTurret turret;
    Follower follower;
    GoalSelector goal;

    @Override
    public void runOpMode() throws InterruptedException {
        // ---------------- Drive + mechanisms ----------------
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");
        DcMotor transfer = hardwareMap.dcMotor.get("transfer");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        Servo hood = hardwareMap.get(Servo.class, "hood");
        Servo gate = hardwareMap.get(Servo.class, "gate");

        DcMotorEx outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeleft");

        outtakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------------- Turret ----------------
        goal = new GoalSelector();
        turret = new EasyTurret();
        follower = Constants.create(hardwareMap);

        goal.setAlliance(ALLIANCE);
        follower.setPose(START_POSE);
        turret.init(hardwareMap, follower, goal);

        double integral = 0;
        double lastError = 0;
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime gateTimer = new ElapsedTime();
        boolean feedPending = false;

        waitForStart();

        if (isStopRequested()) return;

        timer.reset();

        while (opModeIsActive()) {
            // Keep localization and turret tracking running every loop
            follower.update();
            turret.update();

            // ---------------- Mecanum drive ----------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftMotor.setPower((y + x + rx) / denominator);
            backLeftMotor.setPower((y - x + rx) / denominator);
            frontRightMotor.setPower((y - x - rx) / denominator);
            backRightMotor.setPower((y + x - rx) / denominator);

            // ---------------- Mechanism controls ----------------
            if (gamepad1.dpad_up) {
                hood.setPosition(0);
                gate.setPosition(0.);
            }
            if (gamepad1.a) {
                intake.setPower(-1);
                transfer.setPower(1);
            }
            if (gamepad1.left_bumper) {
                outtakeLeft.setPower(-0.01);
                transfer.setPower(0);
                intake.setPower(0);
                gate.setPosition(0.15);
                feedPending = false;
            }
            if (gamepad1.right_bumper) {
                transfer.setPower(-0.8);
                gate.setPosition(0.22);
                intake.setPower(1);
            }
            if (gamepad1.left_trigger_pressed && !feedPending) {
                // Non-blocking version of the old sleep(250) so the turret keeps updating
                gate.setPosition(0);
                gateTimer.reset();
                feedPending = true;
            }
            if (feedPending && gateTimer.seconds() >= GATE_FEED_DELAY_SEC) {
                transfer.setPower(-1);
                intake.setPower(1);
                feedPending = false;
            }
            if (gamepad1.dpad_down) {
                gate.setPosition(0.0);
                hood.setPosition(0.22);
            }
            //

            //reloc
            if (gamepad1.dpad_right) {
                follower.setPose(START_POSE);
            }

            // ---------------- Shooter PID ----------------
            double dt = timer.seconds();
            timer.reset();

            double rawDegPerSec = outtakeLeft.getVelocity(AngleUnit.DEGREES);
            double currentDegPerSec = Math.abs(rawDegPerSec);
            double currentRpm = currentDegPerSec / DEG_PER_SEC_PER_RPM;
            double shooterPower = 0;

            if (gamepad1.right_trigger_pressed) {
                double targetDegPerSec = TARGET_RPM * DEG_PER_SEC_PER_RPM;
                double error = targetDegPerSec - currentDegPerSec;

                if (dt > 0) {
                    integral += error * dt * kI;
                    integral = Range.clip(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
                }

                double derivative = (dt > 0) ? (error - lastError) / dt : 0;
                lastError = error;

                double feedforward = kS + kV * TARGET_RPM;
                shooterPower = feedforward + kP * error + integral + kD * derivative;
                shooterPower = Range.clip(shooterPower, 0, 1);
            } else {
                integral = 0;
                lastError = 0;
            }

            outtakeLeft.setPower(SHOOTER_SIGN * shooterPower);

            // ---------------- Telemetry ----------------
            telemetry.addData("GATE POSITION", gate.getPosition());
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Current RPM", "%.1f", currentRpm);
            telemetry.addData("Raw velocity (deg/s)", "%.1f", rawDegPerSec);
            telemetry.addData("Shooter Power", "%.3f", shooterPower);
            telemetry.addData("Turret Desired Deg", turret.getDesiredDeg());
            telemetry.addData("Turret Current Deg", turret.getCurrentDeg());
            telemetry.addData("Turret Target Deg", turret.getTargetDeg());
            telemetry.addData("Turret Encoder Ticks", turret.getEncoderTicks());
            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.update();
        }

        outtakeLeft.setPower(0);
    }
}