package org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils;


import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.teamcode.RobotConfig;

import java.util.ArrayList;
import java.util.List;

public class HuskyAprilTagCamera implements RawArilTagRecognitionCamera {
    private final HuskyLens camera;
    private ArrayList<AprilTagTargetRaw> visibleTargets;
    private boolean colorMode = false;
    public HuskyAprilTagCamera(HuskyLens camera) {
        this.camera = camera;
    }
    @Override
    public void startRecognizing() {
        camera.initialize();
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        colorMode = false;
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
        for (HuskyLens.Block block:camera.blocks()) {
            if (colorMode &&
                    (block.width < RobotConfig.TeamElementFinderConfigs.minimumSize || block.height < RobotConfig.TeamElementFinderConfigs.minimumSize))
                continue;
            visibleTargets.add(new AprilTagTargetRaw(
                    block.id,
                    block.x - 160,
                    block.y - 120
            ));
        }
    }

    @Override
    public List<AprilTagTargetRaw> getRawArilTagTargets() {
        return visibleTargets;
    }

    public void setToDefaultMode() {
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    public void setToColorMode() {
        camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        colorMode = true;
    }
}
