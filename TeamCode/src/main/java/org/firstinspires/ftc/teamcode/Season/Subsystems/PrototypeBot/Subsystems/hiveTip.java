package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

import dev.nextftc.robot.Mechanism;

public class hiveTip implements Mechanism {
    Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

    public void init() {
        limelight.pipelineSwitch(1);  // Use pipeline slot 0 by default
        limelight.start();
    }

    public final List<Integer> Redfront = List.of(34, 35, 36, 37);
    public final List<Integer> Redback = List.of(30, 31, 32, 33);
    public final List<Integer> Bluefront = List.of(38, 39, 40, 41);
    public final List<Integer> Blueback = List.of(42, 43, 44, 45);

    public int tagId = 0;
    public boolean limelightOn = true;
    public boolean blueFront = false;
    public boolean redFront = false;

    public void checkRed() {
        redFront = Redfront.contains(tagId) || Redback.contains(tagId);
    }

    public void checkBlue() {
        blueFront = Bluefront.contains(tagId) || Blueback.contains(tagId);
    }

    @Override
    public void periodic() {
        Mechanism.super.periodic();
        if (limelightOn && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                if (fiducialResults != null && !fiducialResults.isEmpty()) {
                    tagId = fiducialResults.get(0).getFiducialId();
                } else {
                    tagId = 0;
                }
            } else {
                tagId = 0;
            }
        }

        checkRed();
        checkBlue();

        telemetry.addData("tagId", tagId);
        telemetry.addData("redFront", redFront);
        telemetry.addData("blueFront", blueFront);
    }
}
