//package org.firstinspires.ftc.teamcode.Season.Prototypes;
//
//import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot.ProtoBot;
//
//import dev.nextftc.robot.opmode.NextOpMode;
//import dev.nextftc.robot.opmode.NextTeleop;
//
//@NextTeleop
//public class ColorTest extends NextOpMode {
//
//    ProtoBot robot;
//
//    public ColorTest(ProtoBot robot) {
//        super(robot);
//        this.robot = robot;
//    }
//
//    @Override
//    public void start() {
//        telemetry.addData("Status", "ColorTest Started");
//        robot.colorsensor.init(hardwareMap);
//    }
//
//    @Override
//    public void periodic() {
//        robot.colorsensor.periodic();
//        telemetry.addData("Hue", robot.colorsensor.getHue());
//        telemetry.addData("Sat", robot.colorsensor.getSat());
//        telemetry.addData("Val", robot.colorsensor.getVal());
//        telemetry.addData("Pollen", robot.colorsensor.pollen);
//        telemetry.addData("Red Nectar", robot.colorsensor.red_nectar);
//        telemetry.addData("Blue Nectar", robot.colorsensor.blue_nectar);
//        telemetry.update();
//    }
//}