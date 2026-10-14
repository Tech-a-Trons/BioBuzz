package org.firstinspires.ftc.teamcode.Season.Auto.pedro;

import static org.firstinspires.ftc.teamcode.Season.Auto.pedro.Constants.drivetrainConfig;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.Season.Auto.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.Season.Auto.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.Season.Auto.pedro.procedures.OctoQuadTuner;
import org.firstinspires.ftc.teamcode.Season.Auto.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.Season.Auto.pedro.procedures.Tests;

public class Tuning {

    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, drivetrainConfig), (hardwareMap -> new OctoQuadLocalizer(hardwareMap, Constants.localizerConfig)), () -> new Foresight(Constants.foresightConfig));
    }

    //For tests, finish other config, for foresight, then finish mech (mech fin) and pinpt

//    @Tuner
//    public static Procedure tests() {
//        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), (hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig)), () -> new Foresight(Constants.foresightConfig));
//    }
//

    @Tuner
    public static Procedure octoquadTuner() {
        return new OctoQuadTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner((hardwareMap) -> new OctoQuadLocalizer(hardwareMap, Constants.localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, drivetrainConfig));
    }

}