package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.Telemetry;

public class hiveTip implements Mechanism {
    public Limelight3A limelight;

    public void init(HardwareMap hardwareMap) {
        if (limelight == null && hardwareMap != null) {
            try {
                limelight = hardwareMap.get(Limelight3A.class, "limelight");
            } catch (Exception e) {
                Telemetry.log("hiveTip Error", "Limelight device 'limelight' not found in hardwareMap!");
            }
        }
        if (limelight != null) {
            try {
                limelight.pipelineSwitch(1);
                limelight.start();
            } catch (Exception e) {
                Telemetry.log("hiveTip Error", "Failed to start Limelight: " + e.getMessage());
            }
        }
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

        Telemetry.log("tagId", String.valueOf(tagId));
        Telemetry.log("redFront", String.valueOf(redFront));
        Telemetry.log("blueFront", String.valueOf(blueFront));
    }
}
