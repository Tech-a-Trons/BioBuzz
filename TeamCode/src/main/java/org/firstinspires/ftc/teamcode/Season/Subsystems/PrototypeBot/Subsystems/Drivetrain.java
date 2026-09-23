package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Drivetrain implements Mechanism {
    public final NextMotor frontLeft = new NextMotor("fl");
    public final NextMotor frontRight = new NextMotor("fr");
    public final NextMotor backLeft = new NextMotor("bl");
    public final NextMotor backRight = new NextMotor("br");

    public void startDrive(Gamepad gamepad) {
        infinite(() -> {
            double forward = -gamepad.right_stick_x; // left stick Y: forward/back
            double strafe  =  gamepad.left_stick_x; // left stick X: strafe
            double turn    =  -gamepad.left_stick_y*-1; // right stick X: turn

            double denom = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);

            frontLeft.setThrottle((forward + strafe + turn) / denom);
            backLeft.setThrottle((forward - strafe + turn) / denom);
            frontRight.setThrottle((forward - strafe - turn) / denom);
            backRight.setThrottle((forward + strafe - turn) / denom);
        }).schedule();
    }
}