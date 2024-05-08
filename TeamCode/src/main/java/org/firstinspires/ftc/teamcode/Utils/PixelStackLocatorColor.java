package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedSensor;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;

public class PixelStackLocatorColor {
    private final ThreadedSensor spikeLineScannerColor;
    private final Chassis chassis;
    private final PositionEstimator positionEstimator;
    private double pixelStackLocationX;
    private final Vector2D pixelStackLocationDefault;
    public PixelStackLocatorColor(ThreadedSensor spikeLineScannerColor, Chassis chassis, PositionEstimator positionEstimator, Vector2D pixelStackLocationDefault) {
        this.spikeLineScannerColor = spikeLineScannerColor;
        this.chassis = chassis;
        this.positionEstimator = positionEstimator;
        this.pixelStackLocationDefault = pixelStackLocationDefault;
    }

    public void reset() {
        this.pixelStackLocationX = pixelStackLocationDefault.getX();
    }

    public Vector2D getPixelStackLocation() {
        return new Vector2D(new double[] {this.pixelStackLocationX, pixelStackLocationDefault.getY()});
    }

    public SequentialCommandSegment locatePixelStackCommandSegment(Vector2D startingPosition, Vector2D endingPosition) {
        final double colorSensorThreshold = 1000, colorSensorPositionOnRobot = 7;
        return new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(startingPosition, endingPosition),
                this::reset,
                () -> {
                    if (spikeLineScannerColor.getSensorReading() > 1000)
                        this.pixelStackLocationX = positionEstimator.getCurrentPosition().getX() + colorSensorPositionOnRobot;
                },
                () -> {},
                chassis::isCurrentTranslationalTaskComplete,
                () -> new Rotation2D(0),
                () -> new Rotation2D(0)
        );
    }
}
