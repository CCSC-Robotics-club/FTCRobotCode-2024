package org.firstinspires.ftc.teamcode.AutoStages;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.StatisticsUtils;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.PositionEstimator;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.RawArilTagRecognitionCamera;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;

import java.util.concurrent.atomic.AtomicLong;

public class CameraAutoCalibrationVerticalOnly extends AutoStageProgram {
    final double cameraInstallationHeightCM;
    final Vector2D robotStartingPositionToTarget;
    final int targetID;


    private static final double minDistanceToTarget = 10, maxDistanceToTarget = 60;
    private static final int verticalDistancesSamples = 10;
    final double[]
            angleYSamples = new double[verticalDistancesSamples],
            pixelYSamples = new double[verticalDistancesSamples];
    int i = 0;
    public CameraAutoCalibrationVerticalOnly(double cameraInstallationHeightCM, Vector2D robotStartingPositionToTarget, int targetID) {
        super(Robot.Side.RED);
        this.cameraInstallationHeightCM = cameraInstallationHeightCM;
        this.robotStartingPositionToTarget = robotStartingPositionToTarget;
        this.targetID = targetID;
    }
    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, robotStartingPositionToTarget, new Rotation2D(0), Robot.Side.RED, robot.hardwareMap);
        super.commandSegments.add(sequentialCommandFactory.calibratePositionEstimator());

        i = 0;
        for (int currentDistanceSample = 0; currentDistanceSample < verticalDistancesSamples; currentDistanceSample++) {
            final double distance = minDistanceToTarget + currentDistanceSample * (maxDistanceToTarget - minDistanceToTarget) / verticalDistancesSamples;
            super.commandSegments.add(moveToPositionAndMeasure(
                    distance,
                    robot.positionEstimator,
                    robot.chassis,
                    robot.aprilTagCamera.getRawAprilTagCamera(),
                    robot.distanceSensor,
                    i++
            ));
        }

        commandSegments.add(sequentialCommandFactory.justDoIt(
                () -> printResultsToTelemetry(telemetrySender.getTelemetryLegacy())
        ));

        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> {}, () -> {}, () -> {},
                () -> false,
                () -> null, () -> null
        ));
    }

    private SequentialCommandSegment moveToPositionAndMeasure(double distance, PositionEstimator positionEstimator, Chassis chassis, RawArilTagRecognitionCamera cameraToTest, DistanceSensor distanceSensor, int i) {
        final long timeOut = 2000;
        AtomicLong taskStartedTime = new AtomicLong();
        return new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(positionEstimator.getCurrentPosition(), new Vector2D(new double[] {0,-distance})),
                () -> taskStartedTime.set(System.currentTimeMillis()),
                cameraToTest::update,
                () -> {
                    if (System.currentTimeMillis() - taskStartedTime.get() > timeOut)
                        return;
                    angleYSamples[i] = Math.atan(distanceSensor.getDistance(DistanceUnit.CM) / cameraInstallationHeightCM);
                    pixelYSamples[i] = cameraToTest.getRawAprilTagByID(targetID).y;
                },
                () -> System.currentTimeMillis() - taskStartedTime.get() > timeOut ||
                        (chassis.isCurrentTranslationalTaskComplete() && chassis.isCurrentRotationalTaskComplete() && cameraToTest.getRawAprilTagByID(targetID) != null),
                () -> new Rotation2D(0), () -> new Rotation2D(0)
        );
    }

    private void printResultsToTelemetry(Telemetry telemetry) {
        final double
                cameraAngleRadianPerPixelY = StatisticsUtils.getBestFitLineSlope(pixelYSamples, angleYSamples),
                cameraInstallationAngleY = StatisticsUtils.getBestFitLineIntersect(pixelYSamples, angleYSamples);

        telemetry.update();

        telemetry.addData("camera angle radian per pixel (y)", cameraAngleRadianPerPixelY);
        telemetry.addData("'camera installation angle (pitch)", cameraInstallationAngleY);
        telemetry.addData("(y-direction) data correlation squared", Math.pow(StatisticsUtils.getCorrelationCoefficient(pixelYSamples, angleYSamples), 2));

        telemetry.update();

        try {
            Thread.sleep(30000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
