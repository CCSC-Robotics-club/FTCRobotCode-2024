package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

// TODO: make this program work
public class CameraAutoCalibration extends AutoStageProgram {
    final double cameraInstallationHeightCM;
    final Vector2D robotStartingPositionToTarget;
    final int targetID;
    final Rotation2D cameraFacingRotation;


    private static final double minDistanceToTarget = 10, maxDistanceToTarget = 50, maxHorizontalAngle = Math.toRadians(45);
    private static final int horizontalAnglesLevels = 3, verticalDistancesSamples = 5, horizontalAnglesSamples = horizontalAnglesLevels * 2 + 1;
    final double[]
            angleYSamples = new double[horizontalAnglesSamples * verticalDistancesSamples],
            angleXSamples = new double[horizontalAnglesSamples * verticalDistancesSamples],
            pixelYSamples = new double[horizontalAnglesSamples * verticalDistancesSamples],
            pixelXSamples = new double[horizontalAnglesSamples * verticalDistancesSamples];
    int i = 0;
    public CameraAutoCalibration(double cameraInstallationHeightCM, Vector2D robotStartingPositionToTarget, int targetID, Rotation2D cameraFacing) {
        super(Robot.Side.RED);
        this.cameraInstallationHeightCM = cameraInstallationHeightCM;
        this.robotStartingPositionToTarget = robotStartingPositionToTarget;
        this.targetID = targetID;
        this.cameraFacingRotation = cameraFacing;
    }

    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, robotStartingPositionToTarget, new Rotation2D(0), Robot.Side.RED, robot.hardwareMap);
        super.commandSegments.add(sequentialCommandFactory.calibratePositionEstimator());

        i = 0;
        for (int currentDistanceSample = 0; currentDistanceSample < verticalDistancesSamples; currentDistanceSample++) {
            final double distance = minDistanceToTarget + currentDistanceSample * (maxDistanceToTarget - minDistanceToTarget) / verticalDistancesSamples;
            for (int currentAngleSample = 0; currentAngleSample < horizontalAnglesSamples; currentAngleSample++)
                super.commandSegments.add(moveToPositionAndMeasure(
                        distance,
                        -maxHorizontalAngle + currentAngleSample * (maxHorizontalAngle / horizontalAnglesLevels),
                        robot.positionEstimator,
                        robot.chassis,
                        robot.aprilTagCamera.getRawAprilTagCamera(),
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

    private SequentialCommandSegment moveToPositionAndMeasure(double distance, double angle, PositionEstimator positionEstimator, Chassis chassis, RawArilTagRecognitionCamera cameraToTest, int i) {
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
                    final Vector2D targetRelativePositionToRobotFieldOriented = positionEstimator.getCurrentPosition().multiplyBy(-1),
                            targetRelativePositionToRobotInCameraView = targetRelativePositionToRobotFieldOriented.multiplyBy(cameraFacingRotation.getReversal()).multiplyBy(positionEstimator.getRotation2D().getReversal());

                    angleXSamples[i] = targetRelativePositionToRobotInCameraView.getHeading();
                    angleYSamples[i] = Math.atan(targetRelativePositionToRobotInCameraView.getY() / cameraInstallationHeightCM);
                    pixelXSamples[i] = cameraToTest.getRawAprilTagByID(targetID).x;
                    pixelYSamples[i] = cameraToTest.getRawAprilTagByID(targetID).y;
                },
                () -> System.currentTimeMillis() - taskStartedTime.get() > timeOut ||
                        (chassis.isCurrentTranslationalTaskComplete() && chassis.isCurrentRotationalTaskComplete() && cameraToTest.getRawAprilTagByID(targetID) != null),
                positionEstimator::getRotation2D, () -> new Rotation2D(-angle)
        );
    }

    private void printResultsToTelemetry(Telemetry telemetry) {
        final double cameraAngleRadianPerPixelX = StatisticsUtils.getBestFitLineSlope(pixelXSamples, angleXSamples),
                cameraAngleBiasX = StatisticsUtils.getBestFitLineIntersect(pixelXSamples, angleXSamples),
                cameraAngleRadianPerPixelY = StatisticsUtils.getBestFitLineSlope(pixelYSamples, angleYSamples),
                cameraInstallationAngleY = StatisticsUtils.getBestFitLineIntersect(pixelYSamples, angleYSamples);

        telemetry.update();
        telemetry.addData("camera angle radian per pixel (x)", cameraAngleRadianPerPixelX);
        telemetry.addData("camera angle x bias", cameraAngleBiasX);
        telemetry.addData("(x-direction) data correlation squared", Math.pow(StatisticsUtils.getCorrelationCoefficient(pixelXSamples, angleXSamples), 2));

        telemetry.addData("camera angle radian per pixel (y)", cameraAngleRadianPerPixelY);
        telemetry.addData("'camera installation angle (pitch)", cameraInstallationAngleY);
        telemetry.addData("(y-direction) data correlation squared", Math.pow(StatisticsUtils.getCorrelationCoefficient(pixelYSamples, angleYSamples), 2));

        telemetry.addLine("press X to continue");
        telemetry.update();
    }
}
