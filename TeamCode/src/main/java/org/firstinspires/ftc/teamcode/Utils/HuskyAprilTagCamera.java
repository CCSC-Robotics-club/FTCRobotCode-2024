package org.firstinspires.ftc.teamcode.Utils;


import com.qualcomm.hardware.dfrobot.HuskyLens;

import java.util.ArrayList;
import java.util.List;

public class HuskyAprilTagCamera implements RawArilTagRecognitionCamera {
    private final HuskyLens camera;
    private ArrayList<AprilTagTargetRaw> visibleTargets;
    public HuskyAprilTagCamera(HuskyLens camera) {
        this.camera = camera;
    }
    @Override
    public void startRecognizing() {
        camera.initialize();
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    @Override
    public void stopRecognizing() {
        // do nothing
    }

    @Override
    public void update() {
        if (!camera.knock())
            throw new IllegalStateException("no response from husky lens");

        visibleTargets = new ArrayList<>(1);
        for (HuskyLens.Block block:camera.blocks())
            visibleTargets.add(new AprilTagTargetRaw(
                    block.id,
                    block.x - 160,
                    block.y - 120
            ));
    }

    @Override
    public List<AprilTagTargetRaw> getRawArilTagTargets() {
        return visibleTargets;
    }
}
