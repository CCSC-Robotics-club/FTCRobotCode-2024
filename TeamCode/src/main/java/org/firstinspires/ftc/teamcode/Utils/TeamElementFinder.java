package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import static org.firstinspires.ftc.teamcode.RobotConfig.TeamElementFinderConfigs;

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
    public TeamElementFinder(Chassis chassis, DistanceSensor distanceSensor) {
        this.chassis = chassis;
        this.distanceSensor = distanceSensor;

        this.teamElementPosition = TeamElementPosition.UNDETERMINED;
    }

    public SequentialCommandSegment getDistanceSensorFindingCommand(TeamElementPosition positionToSearch, double centerTeamElementRotation) {
        final double startingRotation = Objects.requireNonNull(TeamElementFinderConfigs.teamElementPositionSearchRotationRanges.get(positionToSearch))[0] + centerTeamElementRotation,
                endingRotation = Objects.requireNonNull(TeamElementFinderConfigs.teamElementPositionSearchRotationRanges.get(positionToSearch))[1] + centerTeamElementRotation;
        return new SequentialCommandSegment(
                () -> getFindingResult() == TeamElementPosition.UNDETERMINED,
                null,
                () -> {},
                () -> {
                    if (!AngleUtils.isWithInRange(chassis.getYaw(), startingRotation, endingRotation))
                        return;
                    if (distanceSensor.getDistance(DistanceUnit.CM) < TeamElementFinderConfigs.distanceThreshold)
                        this.teamElementPosition = positionToSearch;
                },
                () -> {},
                () -> getFindingResult() == TeamElementPosition.UNDETERMINED,
                startingRotation, endingRotation
        );
    }

    public TeamElementPosition getFindingResult() {
        return teamElementPosition;
    }
}
