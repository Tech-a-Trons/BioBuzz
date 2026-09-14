package org.firstinspires.ftc.teamcode.Season.Auto.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.Season.Auto.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.Season.Auto.pedro.procedures.PinpointTuner;

public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    //For tests, finish other config, for foresight, then finish mech (mech fin) and pinpt

//    @Tuner
//    public static Procedure tests() {
//        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), (hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig)), () -> new Foresight(Constants.foresightConfig));
//    }
//
//    @Tuner
//    public static Procedure foresightTuner() {
//        return new ForesightTuner((hardwareMap) -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, drivetrainConfig));
//    }

}