package org.firstinspires.ftc.teamcode.Season.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1.EasyTurret;
import org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1.GoalSelector;

@TeleOp
public class EasyTurretOp extends LinearOpMode {

    EasyTurret turret;
    Follower follower;
    GoalSelector goal;

    @Override
    public void runOpMode() throws InterruptedException {
        goal = new GoalSelector();
        turret = new EasyTurret();
        follower = Constants.create(hardwareMap); // your Pedro Constants class

        goal.setAlliance("BLUE");
        follower.setPose(new Pose(94.7, 118.75, Math.toRadians(270)));

        turret.init(hardwareMap, follower, goal);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            turret.update();

            telemetry.addData("Desired Deg", turret.getDesiredDeg());
            telemetry.addData("Current Deg", turret.getCurrentDeg());
            telemetry.addData("Target Deg", turret.getTargetDeg());
            telemetry.update();
        }
    }
}