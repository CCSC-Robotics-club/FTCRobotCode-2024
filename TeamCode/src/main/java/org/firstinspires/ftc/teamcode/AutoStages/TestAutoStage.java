package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AprilTagCameraAndDistanceSensorAimBot;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.PixelStackLocator;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.TeamElementFinder;

public class TestAutoStage extends AutoStageProgram {
    public TestAutoStage(Robot.Side side) {
        super(side);
    }

    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, super.allianceSide, robot.hardwareMap);
        final PixelStackLocator pixelStackLocator = new PixelStackLocator(robot.distanceSensorBack, robot.chassis);
        super.commandSegments.add(sequentialCommandFactory.calibratePositionEstimator());
        super.commandSegments.add(sequentialCommandFactory.moveToPointAndStop(new Vector2D(new double[] {-10, 0})));
        super.commandSegments.add(pixelStackLocator.locatePixelPosition(
                new BezierCurve(new Vector2D(new double[] {-10, 0}), new Vector2D(new double[] {10, 0}))
                ));
        super.commandSegments.add(sequentialCommandFactory.justDoIt(() -> {
            telemetrySender.putSystemMessage("pixel stack location (x)", pixelStackLocator.getPixelStackLocation().getX());
            telemetrySender.putSystemMessage("pixel stack location (y)", pixelStackLocator.getPixelStackLocation().getY());
        }));
        super.commandSegments.add(sequentialCommandFactory.waitFor(50000));
    }
}
