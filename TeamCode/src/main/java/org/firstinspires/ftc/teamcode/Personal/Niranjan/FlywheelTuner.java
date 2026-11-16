package org.firstinspires.ftc.teamcode.Personal.Niranjan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

/**
 * Flywheel tuner. Two modes, toggled with BACK:
 *
 * OPEN LOOP (find kS and kV, no PID)
 *   dpad up/down  : power +/- 0.05      dpad right/left : power +/- 0.01
 *   Y : save current power as kS  (do this when the wheel JUST starts turning)
 *   X : record a sample (power, settled deg/s) -> kV is averaged from all samples
 *   right bumper : apply the averaged kV
 *   B : clear samples      A : power to 0
 *
 * CLOSED LOOP (feedforward + PID, tune kP, then kI/kD if needed)
 *   A : spin on/off
 *   dpad up/down : target RPM +/- 100     dpad right/left : +/- 10
 *   left/right bumper : select gain (kS, kV, kP, kI, kD)
 *   X : selected gain x1.25      B : selected gain /1.25
 *   Y : reset the integral
 *
 * The constants to paste into FlywheelShooter are shown in telemetry at all times.
 */
@TeleOp(name = "Flywheel Tuner", group = "Tuning")
public class FlywheelTuner extends LinearOpMode {
    // Same direction convention as FlywheelShooter
    private static final double LEFT_SIGN = -1;
    private static final double RIGHT_SIGN = 1;

    private static final double DEG_PER_SEC_PER_RPM = 6.0;
    private static final double MAX_INTEGRAL = 0.2;
    private static final double GAIN_STEP = 1.25;

    private static final int KS = 0, KV = 1, KP = 2, KI = 3, KD = 4;
    private static final String[] NAMES = {"kS", "kV", "kP", "kI", "kD"};
    // If a gain is 0, the first "up" press sets it to this value
    private static final double[] SEEDS = {0.005, 1e-6, 1e-6, 1e-7, 1e-7};

    private final double[] gains = {0.02, 0.000027, 0.00003, 0.0, 0.0};

    private final boolean[] prev = new boolean[12];
    private boolean edge(boolean cur, int i) {
        boolean pressed = cur && !prev[i];
        prev[i] = cur;
        return pressed;
    }

    @Override
    public void runOpMode() {
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "outtakeleft");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "outtakeright");
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        boolean closedLoop = false;
        double openPower = 0;
        ArrayList<double[]> samples = new ArrayList<>(); // {power, deg/s}

        boolean spinning = false;
        double targetRpm = 700;
        int sel = KP;
        double integral = 0, lastError = 0;

        ElapsedTime timer = new ElapsedTime();

        telemetry.addLine("Flywheel Tuner ready. BACK toggles OPEN/CLOSED loop.");
        telemetry.update();
        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double dt = timer.seconds();
            timer.reset();

            double vel = Math.abs(left.getVelocity(AngleUnit.DEGREES)); // deg/s, left encoder only
            double rpm = vel / DEG_PER_SEC_PER_RPM;
            double battery = hardwareMap.voltageSensor.iterator().next().getVoltage();

            if (edge(gamepad1.back, 0)) {
                closedLoop = !closedLoop;
                openPower = 0;
                spinning = false;
                integral = 0;
                lastError = 0;
            }

            double power = 0;

            if (!closedLoop) {
                // ---------------- OPEN LOOP: find kS and kV ----------------
                if (edge(gamepad1.dpad_up, 1))    openPower += 0.05;
                if (edge(gamepad1.dpad_down, 2))  openPower -= 0.05;
                if (edge(gamepad1.dpad_right, 3)) openPower += 0.01;
                if (edge(gamepad1.dpad_left, 4))  openPower -= 0.01;
                if (edge(gamepad1.a, 5))          openPower = 0;
                openPower = Range.clip(openPower, 0, 1);

                if (edge(gamepad1.y, 6)) gains[KS] = openPower;
                if (edge(gamepad1.x, 7) && vel > 50) samples.add(new double[]{openPower, vel});
                if (edge(gamepad1.b, 8)) samples.clear();

                double suggestedKv = 0;
                int n = 0;
                for (double[] s : samples) {
                    suggestedKv += (s[0] - gains[KS]) / s[1];
                    n++;
                }
                if (n > 0) suggestedKv /= n;
                if (edge(gamepad1.right_bumper, 9) && n > 0) gains[KV] = suggestedKv;

                power = openPower;

                telemetry.addData("MODE", "OPEN LOOP (BACK = closed loop)");
                telemetry.addData("power", "%.2f", openPower);
                telemetry.addData("deg/s", "%.0f", vel);
                telemetry.addData("RPM", "%.0f", rpm);
                telemetry.addData("battery V", "%.2f", battery);
                telemetry.addLine();
                telemetry.addData("kS (Y to set)", "%.4f", gains[KS]);
                telemetry.addData("samples (X to add, B to clear)", n);
                for (int i = Math.max(0, n - 5); i < n; i++) {
                    double[] s = samples.get(i);
                    telemetry.addData("  sample " + i, "p=%.2f  %.0f deg/s  kV=%.7f",
                            s[0], s[1], (s[0] - gains[KS]) / s[1]);
                }
                telemetry.addData("suggested kV (RB to apply)", "%.8f", suggestedKv);
                telemetry.addData("kV in use", "%.8f", gains[KV]);
            } else {
                // ---------------- CLOSED LOOP: tune kP etc. ----------------
                if (edge(gamepad1.a, 5)) {
                    spinning = !spinning;
                    integral = 0;
                    lastError = 0;
                }
                if (edge(gamepad1.dpad_up, 1))    targetRpm += 100;
                if (edge(gamepad1.dpad_down, 2))  targetRpm -= 100;
                if (edge(gamepad1.dpad_right, 3)) targetRpm += 10;
                if (edge(gamepad1.dpad_left, 4))  targetRpm -= 10;
                targetRpm = Math.max(0, targetRpm);

                if (edge(gamepad1.right_bumper, 9)) sel = (sel + 1) % NAMES.length;
                if (edge(gamepad1.left_bumper, 10)) sel = (sel + NAMES.length - 1) % NAMES.length;
                if (edge(gamepad1.x, 7)) {
                    gains[sel] = (gains[sel] == 0) ? SEEDS[sel] : gains[sel] * GAIN_STEP;
                }
                if (edge(gamepad1.b, 8)) {
                    double v = gains[sel] / GAIN_STEP;
                    gains[sel] = (v < SEEDS[sel] * 0.5) ? 0 : v;
                }
                if (edge(gamepad1.y, 6)) integral = 0;

                double targetDeg = targetRpm * DEG_PER_SEC_PER_RPM;
                double error = targetDeg - vel;
                double ff = 0, pTerm = 0, dTerm = 0;

                if (spinning) {
                    if (dt > 0 && Math.abs(error) < 300) {
                        integral = Range.clip(integral + error * dt * gains[KI], -MAX_INTEGRAL, MAX_INTEGRAL);
                    } else {
                        integral = 0;
                    }
                    double derivative = (dt > 0) ? (error - lastError) / dt : 0;
                    lastError = error;

                    ff = gains[KS] + gains[KV] * targetDeg;
                    pTerm = gains[KP] * error;
                    dTerm = gains[KD] * derivative;
                    power = Range.clip(ff + pTerm + integral + dTerm, 0, 1);
                } else {
                    integral = 0;
                    lastError = 0;
                }

                telemetry.addData("MODE", "CLOSED LOOP (BACK = open loop)");
                telemetry.addData("spinning (A)", spinning);
                telemetry.addData("target RPM", "%.0f", targetRpm);
                telemetry.addData("current RPM", "%.0f", rpm);
                telemetry.addData("error RPM", "%.0f", targetRpm - rpm);
                telemetry.addData("power", "%.3f", power);
                telemetry.addData("  feedforward part", "%.3f", ff);
                telemetry.addData("  P part", "%.3f", pTerm);
                telemetry.addData("  I part", "%.3f", integral);
                telemetry.addData("  D part", "%.3f", dTerm);
                telemetry.addData("battery V", "%.2f", battery);
                telemetry.addLine();
                telemetry.addData("selected gain (bumpers)", NAMES[sel]);
                telemetry.addLine("X = x1.25, B = /1.25");
            }

            // Both motors get the same power; the left's encoder is the only feedback
            left.setPower(LEFT_SIGN * power);
            right.setPower(RIGHT_SIGN * power);

            telemetry.addLine();
            telemetry.addLine("---- copy into FlywheelShooter ----");
            telemetry.addData("kS", "%.5f", gains[KS]);
            telemetry.addData("kV", "%.8f", gains[KV]);
            telemetry.addData("kP", "%.8f", gains[KP]);
            telemetry.addData("kI", "%.9f", gains[KI]);
            telemetry.addData("kD", "%.9f", gains[KD]);
            telemetry.update();
        }

        left.setPower(0);
        right.setPower(0);
    }
}