package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TeamElementFinder {
    final String[] LABELS = {
            "team-prop-red",
    };
    final TfodProcessor tfod = new TfodProcessor.Builder().setModelAssetName("Team Prop Red.tflite").setModelLabels(LABELS).build();
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
    public TeamElementFinder(WebcamName webcamName) {
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
        visionPortal.setProcessorEnabled(tfod, false);
        tfod.shutdown();
    }

    public TeamElementPosition getTeamElementPosition() {
        return teamElementPosition;
    }
}
