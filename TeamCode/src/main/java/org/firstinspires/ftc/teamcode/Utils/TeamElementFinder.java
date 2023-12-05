package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.RobotConfig;

public class TeamElementFinder {
    public enum TeamElementPosition {
        LEFT,
        CENTER,
        RIGHT,
        UNDETERMINED
    }

    private final Chassis chassis;
    private final DistanceSensor distanceSensor;
    private final RawArilTagRecognitionCamera aprilTagCameraRaw;
    private TeamElementPosition teamElementPosition;
    public TeamElementFinder(Chassis chassis, DistanceSensor distanceSensor, RawArilTagRecognitionCamera aprilTagCameraRaw) {
        this.chassis = chassis;
        this.distanceSensor = distanceSensor;
        this.aprilTagCameraRaw = aprilTagCameraRaw;

        this.teamElementPosition = TeamElementPosition.UNDETERMINED;
    }

    public TeamElementPosition findElementWithAprilTagCamera() {
        RawArilTagRecognitionCamera.AprilTagTargetRaw aprilTagTargetRaw = aprilTagCameraRaw.getRawAprilTagByID(RobotConfig.VisualNavigationConfigs.teamElementAprilTagID);
        if (aprilTagTargetRaw == null)
            return this.teamElementPosition;
        Vector2D teamElementPositionRaw = new Vector2D(new double[] {aprilTagTargetRaw.x, aprilTagTargetRaw.y});
        final double differenceToLeftPosition = Vector2D.displacementToTarget(teamElementPositionRaw, RobotConfig.VisualNavigationConfigs.teamElementPositionLeft).getMagnitude(),
                differenceToCenterPosition = Vector2D.displacementToTarget(teamElementPositionRaw, RobotConfig.VisualNavigationConfigs.teamElementPositionCenter).getMagnitude(),
                differenceToRightPosition = Vector2D.displacementToTarget(teamElementPositionRaw, RobotConfig.VisualNavigationConfigs.teamElementPositionRight).getMagnitude(),
                minDifference = Math.min(Math.min(differenceToLeftPosition, differenceToRightPosition), differenceToCenterPosition);

        if (minDifference == differenceToLeftPosition)
            return this.teamElementPosition = TeamElementPosition.LEFT;
        if (minDifference == differenceToCenterPosition)
            return this.teamElementPosition = TeamElementPosition.CENTER;
        if (minDifference == differenceToRightPosition)
            return this.teamElementPosition = TeamElementPosition.RIGHT;
        return teamElementPosition;
    }

    public SequentialCommandSegment getDistanceSensorFindingCommand() {
        return new SequentialCommandSegment(
                null,
                () -> {

                },
                () -> {

                },
                () -> {

                },
                () -> true,
                0, 0
        ); // TODO write this part
    }

    public TeamElementPosition getFindingResult() {
        return teamElementPosition;
    }
}
