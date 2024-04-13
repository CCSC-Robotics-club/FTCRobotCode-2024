package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.SpeedCurves;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class PixelStackLocator {
    private static final double distanceTolerance = 4.0;

    private final DistanceSensor distanceSensorBack;
    private final Chassis chassis;
    public PixelStackLocator(DistanceSensor distanceSensorBack, Chassis chassis) {
        this.distanceSensorBack = distanceSensorBack;
        this.chassis = chassis;
    }

    private Vector2D pixelStackLocation = null;

    public SequentialCommandSegment locatePixelPosition(BezierCurve movementPath) {
        final List<Vector2D> recordedPositions = new ArrayList<>();
        return new SequentialCommandSegment(
                () -> true,
                () -> movementPath,
                recordedPositions::clear,
                () -> {
                    final double distanceSensorBackReading = distanceSensorBack.getDistance(DistanceUnit.CM);
                    if (2 < distanceSensorBackReading && distanceSensorBackReading < 30)
                        recordedPositions.add(chassis.getChassisEncoderPosition().addBy(new Vector2D(new double[] {0, -distanceSensorBack.getDistance(DistanceUnit.CM)})));
                },
                () -> this.analyzeResults(recordedPositions),
                chassis::isCurrentTranslationalTaskComplete,
                () -> new Rotation2D(0), () -> new Rotation2D(0),
                SpeedCurves.originalSpeed, 0.4
        );
    }

    private void analyzeResults(List<Vector2D> recordedPositions) {
        double wallPositionY = Double.POSITIVE_INFINITY;
        for (Vector2D recordedPosition:recordedPositions)
            wallPositionY = Math.min(wallPositionY, recordedPosition.getY());

        double stackPositionXEstimate = 0, n = 0;
        for (Vector2D recordedPosition:recordedPositions) {
            if (recordedPosition.getY() - wallPositionY < distanceTolerance)
                continue;
            stackPositionXEstimate += recordedPosition.getX();
            n++;
        }

        if (n==0)
            pixelStackLocation = null;
        else
            pixelStackLocation = new Vector2D(new double[] {stackPositionXEstimate / n, wallPositionY});
    }

    public Vector2D getPixelStackLocation() {
        return pixelStackLocation;
    }
}
