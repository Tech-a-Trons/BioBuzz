package org.firstinspires.ftc.teamcode.Personal.Niranjan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Shooter Diagnostic", group = "Test")
public class ShooterDiagnostic extends LinearOpMode {

    // Same direction sign as MecanumTeleOp
    private static final double SHOOTER_SIGN = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "outtakeleft");
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double testPower = 0.5;
        boolean lastUp = false, lastDown = false;

        waitForStart();

        while (opModeIsActive()) {
            // D-pad up/down changes test power by 0.05
            if (gamepad1.dpad_up && !lastUp) testPower += 0.05;
            if (gamepad1.dpad_down && !lastDown) testPower -= 0.05;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            testPower = Range.clip(testPower, 0, 1);

            // Hold Y to spin in the shooter direction at testPower
            double applied = gamepad1.y ? SHOOTER_SIGN * testPower : 0;
            left.setPower(applied);

            double raw = left.getVelocity(AngleUnit.DEGREES);

            telemetry.addLine("D-pad up/down = change power, hold Y = spin");
            telemetry.addData("Test power", "%.2f", testPower);
            telemetry.addData("Applied power", "%.2f", applied);
            telemetry.addData("Raw velocity (deg/s)", "%.1f", raw);
            telemetry.addData("RPM (magnitude)", "%.1f", Math.abs(raw) / 6.0);
            telemetry.addData("Position (ticks)", left.getCurrentPosition());
            telemetry.update();
        }

        left.setPower(0);
    }
}