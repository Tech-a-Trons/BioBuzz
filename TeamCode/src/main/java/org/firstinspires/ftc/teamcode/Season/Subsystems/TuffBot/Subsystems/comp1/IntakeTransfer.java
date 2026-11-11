package org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.robot.Mechanism;

/** Intake + transfer pair. Powers latch; hardware is only written when a power changes. */
public class IntakeTransfer implements Mechanism {
    private DcMotor intake, transfer;
    private double lastIntake = 2, lastTransfer = 2;

    public void init(HardwareMap hardwareMap) {
        intake   = hardwareMap.dcMotor.get("intake");
        transfer = hardwareMap.dcMotor.get("transfer");
    }

    private void set(double i, double t) {
        if (i != lastIntake)   { intake.setPower(i);   lastIntake = i; }
        if (t != lastTransfer) { transfer.setPower(t); lastTransfer = t; }
    }

    // Powers taken from the simple TurretOp: set(intake, transfer)
    public Command collect()  { return instant(() -> set(-1, 1)); }     // gamepad a
    public Command feed()     { return instant(() -> set(1, -1)); }     // left trigger (after gate opens)
    public Command feedSlow() { return instant(() -> set(1, -0.8)); }   // right bumper
    public Command stop()     { return instant(() -> set(0, 0)); }      // left bumper
}