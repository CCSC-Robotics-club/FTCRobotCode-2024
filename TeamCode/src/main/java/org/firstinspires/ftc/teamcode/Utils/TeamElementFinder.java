package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;

import static org.firstinspires.ftc.teamcode.RobotConfig.TeamElementFinderConfigs;

import java.util.List;
import java.util.Objects;

public class TeamElementFinder {
    public enum TeamElementPosition {
        LEFT,
        CENTER,
        RIGHT,
        UNDETERMINED
    }

    private final Chassis chassis;
    private final DistanceSensor distanceSensor;
    private TeamElementPosition teamElementPosition;
    private final HuskyAprilTagCamera huskyCamera;
    public TeamElementFinder(Chassis chassis, DistanceSensor distanceSensor, HuskyAprilTagCamera huskyCamera) {
        this.chassis = chassis;
        this.distanceSensor = distanceSensor;
        this.huskyCamera = huskyCamera;

        this.teamElementPosition = TeamElementPosition.UNDETERMINED;
    }

    @Deprecated
    public SequentialCommandSegment getDistanceSensorFindingCommand(TeamElementPosition positionToSearch, double centerTeamElementRotation, TelemetrySender telemetrySender) {
        final double startingRotation = Objects.requireNonNull(TeamElementFinderConfigs.teamElementPositionSearchRotationRanges.get(positionToSearch))[0] + centerTeamElementRotation,
                endingRotation = Objects.requireNonNull(TeamElementFinderConfigs.teamElementPositionSearchRotationRanges.get(positionToSearch))[1] + centerTeamElementRotation;
        return new SequentialCommandSegment(
                () -> getFindingResult() == TeamElementPosition.UNDETERMINED,
                null,
                () -> {},
                () -> {
//                    if (!AngleUtils.isWithInRange(chassis.getYaw(), startingRotation, endingRotation))
//                        return;
                    if (distanceSensor.getDistance(DistanceUnit.CM) < TeamElementFinderConfigs.distanceThreshold)
                        this.teamElementPosition = positionToSearch;
                    telemetrySender.putSystemMessage("distance at element position", distanceSensor.getDistance(DistanceUnit.CM));
                },
                () -> {},
                () -> getFindingResult() == TeamElementPosition.UNDETERMINED,
                startingRotation, endingRotation
        );
    }

    public void proceedSearch(TeamElementPosition teamElementPosition) {
        final boolean flag = getResultWithHuskyLens();
        if (flag) this.teamElementPosition = teamElementPosition;
    }

    public void stopSearch() {
        huskyCamera.setToDefaultMode();
    }

    public void startSearch() {
        huskyCamera.setToColorMode();
    }
    private boolean getResultWithHuskyLens(){
        final List<RawArilTagRecognitionCamera.AprilTagTargetRaw> results = huskyCamera.getRawArilTagTargets();
        if (results.isEmpty())
            return false;
        final RawArilTagRecognitionCamera.AprilTagTargetRaw targetRaw = results.get(0);
        final Vector2D targetPosition = new Vector2D(new double[] {targetRaw.x, targetRaw.y}).addBy(TeamElementFinderConfigs.expectedTargetPosition.multiplyBy(-1));

        return targetPosition.getMagnitude() < TeamElementFinderConfigs.searchRangePixels;
    }

    public TeamElementPosition getFindingResult() {
        return teamElementPosition;
    }
}
