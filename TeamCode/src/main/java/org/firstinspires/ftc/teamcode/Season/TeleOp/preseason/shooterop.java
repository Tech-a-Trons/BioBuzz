//package org.firstinspires.ftc.teamcode.Season.TeleOp;
//
//import static com.pedropathing.ivy.groups.Groups.parallel;
//
//import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot.ProtoBot;
//
//import dev.nextftc.robot.Telemetry;
//import dev.nextftc.robot.opmode.NextOpMode;
//import dev.nextftc.robot.opmode.NextTeleop;
//import dev.nextftc.robot.triggers.CommandGamepad;
//
//@NextTeleop
//public class shooterop extends NextOpMode {
//
//    ProtoBot robot;
//
//    public shooterop(ProtoBot robot) {
//        super(robot);
//        this.robot = robot;
//    }
//
//    @Override
//    public void disabledPeriodic() {
//        Telemetry.log("Status", "Init");
//    }
//
//    @Override
//    public void start() {
//        CommandGamepad gp1 = new CommandGamepad(gamepad1);
//        CommandGamepad gp2 = new CommandGamepad(gamepad2);
//        Telemetry.log("Status", "Start");
//        gp1.rightTrigger().isBetween(0.05, 1.0).whileTrue(robot.shoot.run(50));
//    }
//
//    @Override
//    public void periodic() {
//        Telemetry.log("Status", "Running");
//    }
//
//    @Override
//    public void end() {
//        Telemetry.log("Status", "End");
//    }
//}