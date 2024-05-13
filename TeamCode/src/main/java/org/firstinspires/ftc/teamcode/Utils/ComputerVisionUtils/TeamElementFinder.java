package org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TeamElementFinder {
    final String[] LABELS;
    final TfodProcessor tfod;
    final VisionPortal visionPortal;
    public enum TeamElementPosition {
        LEFT,
        CENTER,
        RIGHT,
        UNDETERMINED
    }

    private static final Map<TeamElementPosition, Integer> teamElementPositionsPixel = new HashMap<>();
    static {
        teamElementPositionsPixel.put(TeamElementPosition.LEFT, 60);
        teamElementPositionsPixel.put(TeamElementPosition.CENTER, 340);
        teamElementPositionsPixel.put(TeamElementPosition.RIGHT, 580);
    }

    private TeamElementPosition teamElementPosition;
    public TeamElementFinder(WebcamName webcamName, Robot.Side side) {
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
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.addProcessor(tfod);

        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(tfod, true);
        this.teamElementPosition = TeamElementPosition.UNDETERMINED;
    }

    public void findTeamElementOnce() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        if (!(currentRecognitions.size() == 1 && currentRecognitions.get(0).getLabel().contains("team-prop")))
            return;

        double minDistance = 99999, targetPosition = (currentRecognitions.get(0).getLeft() + currentRecognitions.get(0).getRight())/2;
        for (TeamElementPosition position:teamElementPositionsPixel.keySet()) {
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
                () -> teamElementPosition != TeamElementPosition.UNDETERMINED || System.currentTimeMillis() - startTime[0] > timeOut,
                () -> null, () -> null
        );
    }

    public void shutDown() {
        // visionPortal.setProcessorEnabled(tfod, false);
    }

    public TeamElementPosition getTeamElementPosition() {
        return teamElementPosition;
    }
}
