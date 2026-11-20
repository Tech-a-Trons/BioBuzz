
package org.firstinspires.ftc.teamcode.Season.SensorStuff.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.Telemetry;

public class hiveTip implements Mechanism {

    public Limelight3A limelight;

    private static final int PIPELINE = 1;
    private static final long MAX_STALENESS_MS = 250;
    private static final long POLL_INTERVAL_NS = 20_000_000L;

    private long lastPollNs = 0;

    public final List<Integer> Redfront =
            Arrays.asList(34, 35, 36, 37);

    public final List<Integer> Redback =
            Arrays.asList(30, 31, 32, 33);

    public final List<Integer> Bluefront =
            Arrays.asList(38, 39, 40, 41);

    public final List<Integer> Blueback =
            Arrays.asList(42, 43, 44, 45);

    public int tagId = 0;
    public boolean limelightOn = false;
    public boolean blueFront = false;
    public boolean redFront = false;

    public String status = "Not initialized";
    public long resultAgeMs = -1;
    public long updateCount = 0;

    // INITIALIZATION
    public void init(HardwareMap hardwareMap) {

        if (limelight != null) {
            return;
        }

        if (hardwareMap == null) {
            status = "HardwareMap unavailable";
            return;
        }

        try {
            limelight = hardwareMap.get(
                    Limelight3A.class,
                    "limelight"
            );

            limelight.setPollRateHz(50);
            limelight.pipelineSwitch(PIPELINE);
            limelight.start();

            limelightOn = true;
            status = "Started";

            Telemetry.log(
                    "hiveTip",
                    "Limelight started on pipeline 1"
            );

        } catch (Exception e) {
            limelightOn = false;
            status = "Initialization failed";

            Telemetry.log(
                    "hiveTip Error",
                    String.valueOf(e.getMessage())
            );
        }
    }

    // UPDATE TAG ID FROM LIMELIGHT
    public void updateTag() {

        long now = System.nanoTime();

        // Avoid duplicate reads within one robot loop
        if (now - lastPollNs < POLL_INTERVAL_NS) {
            return;
        }

        lastPollNs = now;
        updateCount++;

        // Reset detection values
        tagId = 0;
        redFront = false;
        blueFront = false;
        resultAgeMs = -1;

        if (!limelightOn || limelight == null) {
            status = "Limelight not initialized";
            return;
        }

        try {
            if (!limelight.isConnected()) {
                status = "Limelight disconnected";
                return;
            }

            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                status = "No valid result";
                return;
            }

            resultAgeMs = result.getStaleness();

            // Do not trust old camera data
            if (resultAgeMs < 0 ||
                    resultAgeMs > MAX_STALENESS_MS) {
                status = "Stale camera result";
                return;
            }

            List<LLResultTypes.FiducialResult> tags =
                    result.getFiducialResults();

            if (tags == null || tags.isEmpty()) {
                status = "No AprilTags visible";
                return;
            }

            // Display the first detected tag
            tagId = tags.get(0).getFiducialId();

            // Check all visible tags
            for (LLResultTypes.FiducialResult tag : tags) {

                int id = tag.getFiducialId();

                if (Redfront.contains(id) ||
                        Redback.contains(id)) {
                    redFront = true;
                }

                if (Bluefront.contains(id) ||
                        Blueback.contains(id)) {
                    blueFront = true;
                }
            }

            status = "Tag detected";

        } catch (Exception e) {
            tagId = 0;
            redFront = false;
            blueFront = false;
            status = "Read error: " +
                    e.getClass().getSimpleName();
        }
    }

    // CHECK RED
    public boolean checkRed() {
        updateTag();
        return redFront;
    }

    // CHECK BLUE
    public boolean checkBlue() {
        updateTag();
        return blueFront;
    }

    // Called automatically if registered as a mechanism
    @Override
    public void periodic() {
        updateTag();
    }
}
