package org.firstinspires.ftc.teamcode.Season.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "GateTuner")
public class GateTuner extends LinearOpMode {

    private static final double STEP = 0.05;
    private static final int MAX_STEPS = 20; // 20 * 0.05 = 1.0

    @Override
    public void runOpMode() throws InterruptedException {
        Servo gate = hardwareMap.get(Servo.class, "gate");

        // Track position as an integer step count so floating point error never builds up
        int steps = 0;
        gate.setPosition(0);

        boolean lastUp = false;
        boolean lastDown = false;

        telemetry.addData("Gate Position", 0.0);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            // Only change on the press, not while the button is held
            if (up && !lastUp) {
                steps = Math.min(steps + 1, MAX_STEPS);
            }
            if (down && !lastDown) {
                steps = Math.max(steps - 1, 0);
            }
            lastUp = up;
            lastDown = down;

            double position = steps * STEP;
            gate.setPosition(position);

            telemetry.addData("Gate Position (commanded)", "%.2f", position);
            telemetry.addData("Gate Position (getPosition)", "%.2f", gate.getPosition());
            telemetry.update();
        }
    }
}