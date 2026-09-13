package org.firstinspires.ftc.teamcode.Season.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake TeleOp")
public class Intake extends OpMode {

    private DcMotor intakeMotor;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0.1) {
            intakeMotor.setPower(1.0);
        } else if (gamepad1.left_trigger > 0.1) {
            intakeMotor.setPower(-1.0);
        } else {
            intakeMotor.setPower(0.0);
        }
    }
}