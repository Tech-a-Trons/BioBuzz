package org.firstinspires.ftc.teamcode.Season.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Tunes both ramp servos together.
 * dpad up/down    = +/- 0.05 (coarse)
 * dpad right/left = +/- 0.01 (fine)
 * Y = jump to 0, A = jump to 1
 * Write down the "commanded" value when the ramp is where you want it.
 */
@TeleOp(name = "RampTuner")
public class RampTuner extends LinearOpMode {

    private static final int COARSE = 5;  // 0.05
    private static final int FINE = 1;    // 0.01
    private static final int MAX_STEPS = 100; // 100 * 0.01 = 1.0

    @Override
    public void runOpMode() throws InterruptedException {
        Servo rampRight = hardwareMap.get(Servo.class, "rampright");
        Servo rampLeft = hardwareMap.get(Servo.class, "rampleft");

        // Same reverse as Ramp.java. Change both places if you swap which one is reversed.
        rampLeft.setDirection(Servo.Direction.REVERSE);

        // Integer hundredths so floating point error never builds up
        int steps = 0;
        rampRight.setPosition(0);
        rampLeft.setPosition(0);

        boolean lastUp = false, lastDown = false, lastRight = false, lastLeft = false;

        telemetry.addData("Ramp Position", 0.0);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean right = gamepad1.dpad_right;
            boolean left = gamepad1.dpad_left;

            // Only change on the press, not while held
            if (up && !lastUp) steps += COARSE;
            if (down && !lastDown) steps -= COARSE;
            if (right && !lastRight) steps += FINE;
            if (left && !lastLeft) steps -= FINE;
            lastUp = up;
            lastDown = down;
            lastRight = right;
            lastLeft = left;

            if (gamepad1.y) steps = 0;
            if (gamepad1.a) steps = MAX_STEPS;

            steps = Math.max(0, Math.min(steps, MAX_STEPS));

            double position = steps / 100.0;
            rampRight.setPosition(position);
            rampLeft.setPosition(position);

            telemetry.addData("Ramp Position (commanded)", "%.2f", position);
            telemetry.addData("rampright (getPosition)", "%.2f", rampRight.getPosition());
            telemetry.addData("rampleft (getPosition)", "%.2f", rampLeft.getPosition());
            telemetry.update();
        }
    }
}