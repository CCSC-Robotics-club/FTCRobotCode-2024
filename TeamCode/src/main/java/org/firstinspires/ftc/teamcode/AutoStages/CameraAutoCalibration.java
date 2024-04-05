package org.firstinspires.ftc.teamcode.AutoStages;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Arm;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Modules.FixedAnglePixelCamera;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.SpeedCurves;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.StatisticsUtils;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.PositionEstimator;
import org.firstinspires.ftc.teamcode.Utils.RawArilTagRecognitionCamera;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;

import java.util.concurrent.atomic.AtomicLong;

public class CameraAutoCalibration extends AutoStageProgram {
    final RawArilTagRecognitionCamera cameraToTest;
    final Telemetry telemetry;
    final double cameraInstallationHeightCM;
    final Vector2D robotStartingPositionToTarget;
    final int targetID;


    private static final double minDistanceToTarget = 10, maxDistanceToTarget = 50, maxHorizontalAngle = Math.toRadians(30);
    private static final int horizontalAnglesLevels = 3, verticalDistancesSamples = 5, horizontalAnglesSamples = horizontalAnglesLevels * 2 + 1;
    final double[]
            angleYSamples = new double[horizontalAnglesSamples * verticalDistancesSamples],
            angleXSamples = new double[horizontalAnglesSamples * verticalDistancesSamples],
            pixelYSamples = new double[horizontalAnglesSamples * verticalDistancesSamples],
            pixelXSamples = new double[horizontalAnglesSamples * verticalDistancesSamples];
    int i = 0;
    public CameraAutoCalibration(
            RawArilTagRecognitionCamera cameraToTest, Telemetry telemetry, double cameraInstallationHeightCM, Vector2D robotStartingPositionToTarget, int targetID) {
        super(Robot.Side.RED);
        this.cameraToTest = cameraToTest;
        this.telemetry = telemetry;
        this.cameraInstallationHeightCM = cameraInstallationHeightCM;
        this.robotStartingPositionToTarget = robotStartingPositionToTarget;
        this.targetID = targetID;
    }

    @Override
    public void scheduleCommands(HardwareMap hardwareMap, Chassis chassis, PositionEstimator positionEstimator, DistanceSensor distanceSensor, FixedAngleArilTagCamera angleArilTagCamera, Arm arm, Intake intake, FixedAnglePixelCamera pixelCamera, ModulesCommanderMarker commanderMarker, TelemetrySender telemetrySender) {
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(chassis, positionEstimator, robotStartingPositionToTarget, new Rotation2D(0), Robot.Side.RED, hardwareMap);
        super.commandSegments.add(sequentialCommandFactory.calibratePositionEstimator());

        i = 0;
        for (int currentDistanceSample = 0; currentDistanceSample < verticalDistancesSamples; currentDistanceSample++) {
            final double distance = minDistanceToTarget + currentDistanceSample * (maxDistanceToTarget - minDistanceToTarget);
            for (int currentAngleSample = 0; currentAngleSample < horizontalAnglesSamples; currentAngleSample++)
                super.commandSegments.add(moveToPositionAndMeasure(
                        distance,
                        -maxHorizontalAngle + currentAngleSample * (maxHorizontalAngle / horizontalAnglesLevels),
                        positionEstimator,
                        chassis,
                        i++
                ));
        }

        commandSegments.add(sequentialCommandFactory.justDoIt(this::printResultsToTelemetry));
    }

    private SequentialCommandSegment moveToPositionAndMeasure(double distance, double angle, PositionEstimator positionEstimator, Chassis chassis, int i) {
        final long timeOut = 1000;
        AtomicLong taskStartedTime = new AtomicLong();
        return new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(positionEstimator.getCurrentPosition(), new Vector2D(new double[] {0,-distance})),
                () -> taskStartedTime.set(System.currentTimeMillis()),
                () -> {
                    if (System.currentTimeMillis() - taskStartedTime.get() > timeOut)
                        return;
                    angleYSamples[i] = Math.atan(positionEstimator.getCurrentPosition().getY() / cameraInstallationHeightCM);
                    angleXSamples[i] = -positionEstimator.getRotation();
                    pixelXSamples[i] = cameraToTest.getRawAprilTagByID(targetID).x;
                    pixelYSamples[i] = cameraToTest.getRawAprilTagByID(targetID).y;
                },
                cameraToTest::update,
                () -> System.currentTimeMillis() - taskStartedTime.get() > timeOut ||
                        (chassis.isCurrentTranslationalTaskComplete() && chassis.isCurrentRotationalTaskComplete() && cameraToTest.getRawAprilTagByID(targetID) != null),
                positionEstimator::getRotation2D, () -> new Rotation2D(-angle),
                SpeedCurves.slowDown, 0.5
        );
    }

    private void printResultsToTelemetry() {
        final double cameraAngleRadianPerPixelX = StatisticsUtils.getBestFitLineSlope(pixelXSamples, angleXSamples),
                cameraAngleRadianPerPixelY = StatisticsUtils.getBestFitLineSlope(pixelYSamples, angleYSamples);

        telemetry.update();
        telemetry.addData("camera angle radian per pixel (x)", cameraAngleRadianPerPixelX);
        telemetry.addData("(x-direction) data correlation squared", Math.pow(StatisticsUtils.getCorrelationCoefficient(pixelXSamples, angleXSamples), 2));

        telemetry.addData("camera angle radian per pixel (y)", cameraAngleRadianPerPixelY);
        telemetry.addData("(y-direction) data correlation squared", Math.pow(StatisticsUtils.getCorrelationCoefficient(pixelYSamples, angleYSamples), 2));

        telemetry.addLine("press X to continue");
        telemetry.update();

        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
