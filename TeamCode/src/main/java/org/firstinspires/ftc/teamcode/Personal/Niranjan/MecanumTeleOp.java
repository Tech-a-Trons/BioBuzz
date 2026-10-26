package org.firstinspires.ftc.teamcode.Personal.Niranjan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "ProtoBotCode")
public class MecanumTeleOp extends LinearOpMode {

    // ---- Shooter PID constants (tune these) ----
    // kS and kV were guessed from readings off the old (bad) encoder - re-measure them
    // with ShooterDiagnostic: kS = power where it just starts moving, kV = extra power per RPM
    private static final double kS = 0.12;     // power to overcome friction
    private static final double kV = 0.001;   // power per RPM
    private static final double kP = 0.027;    // error is in deg/s
    private static final double kI = 0.01;      // start at 0, add only if it settles below target
    private static final double kD = 0.03;      // start at 0, add only if it oscillates

    private static final double TARGET_RPM = 34;
    private static final double DEG_PER_SEC_PER_RPM = 6.0; // RPM x 6 = deg/s
    private static final double MAX_INTEGRAL = 0.3;        // anti-windup limit (in power units)

    // Shooter direction is handled here instead of setDirection().
    // The PID runs on positive numbers; this sign only reverses the final motor power.
    // -1 = spin the motor in the "reverse" direction. If the wheel spins the wrong way, change to 1.
    private static final double SHOOTER_SIGN = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");
        DcMotor transfer = hardwareMap.dcMotor.get("transfer");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        Servo hood = hardwareMap.get(Servo.class,"hood");
        Servo gate = hardwareMap.get(Servo.class,"gate");

        // Shooter: we run our own control, so disable the built-in velocity controller.
        // No setDirection() here, so the encoder and motor keep their default signs.
        DcMotorEx outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeleft");
        outtakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double integral = 0;
        double lastError = 0;
        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        timer.reset();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y * 1; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1;  // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * 1;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            if (gamepad1.dpad_up){
                hood.setPosition(0.13);
                gate.setPosition(0.11);

            }
            if (gamepad1.a) {
                intake.setPower(1);
                transfer.setPower(-1);
            }
            if (gamepad1.left_bumper) {
                outtakeLeft.setPower(-0.01);
                transfer.setPower(0);
                intake.setPower(0);
                gate.setPosition(0);
            }
            if (gamepad1.right_bumper) {
                transfer.setPower(-0.8);
                intake.setPower(1);
            }
            if(gamepad1.left_trigger_pressed){
                gate.setPosition(0.09);
                sleep(250);
                transfer.setPower(-1);
                intake.setPower(1);
            }
            if(gamepad1.dpad_down){
                gate.setPosition(0.11);
                hood.setPosition(0);

            }



            // ---------------- Shooter PID ----------------
            double dt = timer.seconds();
            timer.reset();

            double rawDegPerSec = outtakeLeft.getVelocity(AngleUnit.DEGREES);
            // Use the magnitude so the PID always sees a positive speed;
            // the final power below is what gets reversed
            double currentDegPerSec = Math.abs(rawDegPerSec);
            double currentRpm = currentDegPerSec / DEG_PER_SEC_PER_RPM;
            double shooterPower = 0;

            if (gamepad1.right_trigger_pressed) {
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
            } else {
                // Reset controller state when not shooting
                integral = 0;
                lastError = 0;
            }

            // Apply direction here: positive PID power -> spin in the shooter direction
            outtakeLeft.setPower(SHOOTER_SIGN * shooterPower);
            telemetry.addData("GATE POSNIG",gate.getPosition());
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Current RPM", "%.1f", currentRpm);
            telemetry.addData("Raw velocity (deg/s)", "%.1f", rawDegPerSec);
            telemetry.addData("Shooter Power", "%.3f", shooterPower);
            telemetry.update();
        }

        outtakeLeft.setPower(0);
    }
}