package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Season.Auto.pedro.Constants;

public class PranTurret {
    private static final double kS = 0.05;   // friction compensation (tune)
    private static final double kP = 0.02;   // power per degree of error (tune)
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double MAX_INTEGRAL = 0.3;
    private static final double MOTOR_CPR = 537.7;  // encoder ticks per motor output-shaft rotation (your motor's value)
    private static final double GEAR_RATIO = 3.0;   // motor rotations per turret rotation
    private static final double TICKS_PER_DEG = MOTOR_CPR * GEAR_RATIO / 360.0;
    private static final double MIN_DEG = -180, MAX_DEG = 180; // your turret's travel limits

    DcMotorEx turret;
    Follower follower;
    ElapsedTime timer = new ElapsedTime();
    String ialliance = "EMPTY";
    double integral = 0, lastError = 0;

    // Pass in the SAME follower your OpMode updates, don't create a second one
    public void init(HardwareMap hardwareMap, Follower follower, String alliance) {
        this.follower = follower;
        this.ialliance = alliance;

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we're doing our own control
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer.reset();

        follower = Constants.create(hardwareMap);
        follower.update();
    }

    public void update() {
        double dt = timer.seconds();
        timer.reset();

        Pose p = follower.pose(); // use whatever accessor your version has (you used pose())
        double x = p.x(), y = p.y(), heading = p.heading();

        double xt = 0, yt = 0;
        if ("BLUE".equals(ialliance)) { xt = 84; yt = (y >= 70.75) ? 82 : 56; }
        else if ("RED".equals(ialliance)) { xt = 58; yt = (y >= 70.75) ? 82 : 56; }

        // Field angle to target, minus robot heading = turret angle relative to robot
        double fieldAngle = Math.atan2(yt - y, xt - x);
        double targetDeg = Math.toDegrees(AngleUnit.normalizeRadians(fieldAngle - heading));
        targetDeg = Range.clip(targetDeg, MIN_DEG, MAX_DEG);

        double currentDeg = turret.getCurrentPosition() / TICKS_PER_DEG;
        double error = targetDeg - currentDeg;

        if (dt > 0) {
            integral = Range.clip(integral + error * dt * kI, -MAX_INTEGRAL, MAX_INTEGRAL);
        }
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        double power = kP * error + integral + kD * derivative;
        power += Math.signum(error) * kS;
        if (Math.abs(error) < 0.5) power = 0; // deadband to avoid jitter

        turret.setPower(Range.clip(power, -1, 1));
    }

    public Command track() {
        // runs update() every loop; see note below
        return infinite(this::update);
    }
}