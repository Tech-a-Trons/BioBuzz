//package org.firstinspires.ftc.teamcode.Season.TeleOp;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1.Ramp;
//
//@TeleOp(name = "RampZero")
//public class RampZero extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Ramp ramp = new Ramp();
//        ramp.init(hardwareMap);
//
//        // Zero as soon as INIT is pressed so you can line up the horns
//        ramp.setRamp(Ramp.RAMP_ZERO);
//
//        telemetry.addLine("Ramp set to zero");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Keep it at zero (setRamp ignores repeats, so this is cheap)
//            ramp.setRamp(Ramp.RAMP_ZERO);
//
//            telemetry.addData("Ramp Position", "%.2f", ramp.getRampPosition());
//            telemetry.update();
//        }
//    }
//}