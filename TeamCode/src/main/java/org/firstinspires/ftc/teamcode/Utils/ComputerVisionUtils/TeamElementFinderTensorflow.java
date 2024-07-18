package org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils;


import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TeamElementFinderTensorflow {
    final String[] LABELS;
    final TfodProcessor tfod;
    final VisionPortal visionPortal;

    private static final Map<RobotConfig.TeamElementPosition, Integer> teamElementPositionsPixel = new HashMap<>();
    static {
        teamElementPositionsPixel.put(RobotConfig.TeamElementPosition.LEFT, 60);
        teamElementPositionsPixel.put(RobotConfig.TeamElementPosition.CENTER, 340);
        teamElementPositionsPixel.put(RobotConfig.TeamElementPosition.RIGHT, 580);
    }

    public RobotConfig.TeamElementPosition teamElementPosition;
    public TeamElementFinderTensorflow(HardwareMap hardwareMap, Robot.Side side) {
        // final WebcamName webcamName = side == Robot.Side.BLUE ? hardwareMap.get(WebcamName.class, "right") : hardwareMap.get(WebcamName.class, "left");
        final WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        switch (side) {
            case RED: {
                LABELS = new String[] {
                        "team-prop-red"
                };
                tfod = new TfodProcessor.Builder().setModelAssetName("Team Prop Red.tflite").setModelLabels(LABELS).build();
                break;
            }
            case BLUE: {
                LABELS = new String[] {
                    "team-prop-blue"
                };
                tfod =  new TfodProcessor.Builder().setModelAssetName("Team Prop Blue.tflite").setModelLabels(LABELS).build();

                break;
            }
            default: throw new IllegalArgumentException("unknown side: " + side);
        }

        tfod.setMinResultConfidence(0.75f);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(webcamName);
        builder.enableLiveView(true);
        builder.setCameraResolution(new Size(640, 480));
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.addProcessor(tfod);

        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(tfod, true);
        this.teamElementPosition = RobotConfig.TeamElementPosition.UNDETERMINED;
    }

    public void findTeamElementOnce() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        if (currentRecognitions.isEmpty())
            return;
        Recognition highConf = currentRecognitions.get(0);
        for (Recognition recognition: currentRecognitions) {
            if (recognition.getConfidence() > highConf.getConfidence())
                highConf = recognition;
        }

        double minDistance = 99999, targetPosition = (highConf.getLeft() + highConf.getRight())/2;
        for (RobotConfig.TeamElementPosition position:teamElementPositionsPixel.keySet()) {
            if (Math.abs(targetPosition - teamElementPositionsPixel.get(position)) < minDistance) {
                teamElementPosition = position;
                minDistance = Math.abs(targetPosition - teamElementPositionsPixel.get(position));
            }
        }
    }

    public SequentialCommandSegment findTeamElementAndShutDown(long timeOut) {
        final long[] startTime = new long[1];
        return new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> startTime[0] = System.currentTimeMillis(),
                this::findTeamElementOnce,
                this::shutDown,
                () -> teamElementPosition != RobotConfig.TeamElementPosition.UNDETERMINED || System.currentTimeMillis() - startTime[0] > timeOut,
                () -> null, () -> null
        );
    }

    public void shutDown() {
        visionPortal.setProcessorEnabled(tfod, false);
        Thread shutDownThread = new Thread(tfod::shutdown);
        // shutDownThread.start();
    }
}
