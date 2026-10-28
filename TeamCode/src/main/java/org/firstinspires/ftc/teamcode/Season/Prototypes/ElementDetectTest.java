//package org.firstinspires.ftc.teamcode.Season.Prototypes;
//
//import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot.ProtoBot;
//
//import dev.nextftc.robot.Telemetry;
//import dev.nextftc.robot.opmode.NextOpMode;
//import dev.nextftc.robot.opmode.NextTeleop;
//
//@NextTeleop
//public class ElementDetectTest extends NextOpMode {
//    ProtoBot robot;
//
//    public ElementDetectTest(ProtoBot robot) {
//        super(robot);
//        this.robot = robot;
//    }
//
//    @Override
//    public void disabledPeriodic() {
//        Telemetry.log("Status", "ElementDetectTest Init");
//    }
//
//    @Override
//    public void start() {
//        Telemetry.log("Status", "ElementDetectTest Started");
//        robot.colorsensor.init(hardwareMap);
//        robot.laserBreak.init(hardwareMap);
//    }
//
//    @Override
//    public void periodic() {
//        Telemetry.log("Status", "ElementDetectTest Running");
//
//        robot.elementDetect.periodic();
//
//        Telemetry.log("Laser State", String.valueOf(robot.laserBreak.laserInputState));
//        Telemetry.log("Hue", String.valueOf(robot.colorsensor.getHue()));
//        Telemetry.log("Color Counted", String.valueOf(robot.colorsensor.color_counted));
//        Telemetry.log("Pollen Count", String.valueOf(robot.colorsensor.pollen));
//        Telemetry.log("Red Nectar", String.valueOf(robot.colorsensor.red_nectar));
//        Telemetry.log("Blue Nectar", String.valueOf(robot.colorsensor.blue_nectar));
//        Telemetry.log("Ramp Elements", robot.elementDetect.rampElements.toString());
//
//        if (gamepad1.a) {
//            robot.elementDetect.rampClear();
//        }
//    }
//
//    @Override
//    public void end() {
//        Telemetry.log("Status", "ElementDetectTest Ended");
//    }
//}
