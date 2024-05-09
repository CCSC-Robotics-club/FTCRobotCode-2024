package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FlippableDualClaw;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedSensor;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class PixelStackGrabbingCommand {

    public static List<SequentialCommandSegment> getCommandSegmentSegments(Robot robot, SequentialCommandFactory commandFactory, Vector2D stackCenterPositionDefault) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final double clawWidth = 6, grabbingDistanceToWall = 13, colorSensorThreshold = 2000, colorSensorPositionOnRobot = -8, scanningDistanceToWall = 17;

        final Vector2D scanningStartPosition = stackCenterPositionDefault.addBy(new Vector2D(new double[] {20, scanningDistanceToWall}));

        /* step 1: drive a smooth curve to the starting point of the spike-mark detecting process */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(
                        stackCenterPositionDefault.addBy(new Vector2D(new double[] {0, 80})),
                        stackCenterPositionDefault.addBy(new Vector2D(new double[] {0, 50})),
                        scanningStartPosition.addBy(new Vector2D(new double[] {50, 0})),
                        scanningStartPosition
                ),
                () -> {
                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null);
                    robot.claw.setRightClawClosed(true, null);
                    robot.claw.setLeftClawClosed(true, null);
                    robot.extend.setExtendPosition(0, null);
                    robot.arm.setPosition(RobotConfig.ArmConfigs.Position.INTAKE, null);
                },
                () -> {},
                () -> {
                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.PREPARE_TO_GRAB_STACK, null);
                    robot.extend.setExtendPosition(RobotConfig.ExtendConfigs.grabStackValue, null);
                },
                robot.chassis::isCurrentTranslationalTaskComplete,
                () -> new Rotation2D(0),
                () -> new Rotation2D(0)
        ));

        final double startingPositionX = scanningStartPosition.getX();

        /* step 2: move along a horizontal path with distance to wall 17cm (using distance sensor) and find the spike mark */
        final double[] actualStackCenterPositionX = new double[] {stackCenterPositionDefault.getX()};
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> {
                    final double wallPositionY = robot.distanceSensorBack.getSensorReading() < 50 ?
                            robot.positionEstimator.getCurrentPosition().getY() - robot.distanceSensorBack.getSensorReading()
                            : stackCenterPositionDefault.getY();
                    return new BezierCurve(
                            new Vector2D(new double[] {startingPositionX, wallPositionY + scanningDistanceToWall}),
                            new Vector2D(new double[] {0, wallPositionY + scanningDistanceToWall})
                    );
                },
                () -> {},
                () -> {
                    if (robot.spikeMarkDetectionSensor.getSensorReading() > colorSensorThreshold)
                        throw new RuntimeException("spike mark detected!!!");
                        // actualStackCenterPositionX[0] = robot.positionEstimator.getCurrentPosition().getX() + colorSensorPositionOnRobot;
                },
                () -> {},
                robot.chassis::isCurrentTranslationalTaskComplete,
                () -> new Rotation2D(0),
                () -> new Rotation2D(0)
        ));

        /* step 3: line up the lefter claw with the stack */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> {
                    robot.claw.setLeftClawClosed(false, null);
                    robot.claw.setRightClawClosed(false, null);
                },
                () -> {
                    final double wallPositionY = robot.distanceSensorBack.getSensorReading() < 50 ?
                            robot.positionEstimator.getCurrentPosition().getY() - robot.distanceSensorBack.getSensorReading()
                            : stackCenterPositionDefault.getY();
                    robot.chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                            new Vector2D(new double[] {actualStackCenterPositionX[0] - clawWidth, wallPositionY + scanningDistanceToWall})), null);
                },
                () -> {},
                robot.chassis::isCurrentTranslationalTaskComplete,
                () -> new Rotation2D(0),
                () -> new Rotation2D(0)
        ));

        commandSegments.add(commandFactory.waitFor(114514));

        /* step 4: drive back to the grabbing distance */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> {},
                () -> {
                    final double wallPositionY = robot.distanceSensorBack.getSensorReading() < 50 ?
                            robot.positionEstimator.getCurrentPosition().getY() - robot.distanceSensorBack.getSensorReading()
                            : stackCenterPositionDefault.getY();
                    robot.chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                            new Vector2D(new double[] {actualStackCenterPositionX[0] - clawWidth, wallPositionY + grabbingDistanceToWall})), null);
                },
                () -> {},
                robot.chassis::isCurrentTranslationalTaskComplete,
                () -> new Rotation2D(0),
                () -> new Rotation2D(0)
        ));

        commandSegments.add(commandFactory.justDoIt(() -> robot.claw.setFlip(FlippableDualClaw.FlipperPosition.INTAKE, null)));

        commandSegments.add(commandFactory.waitFor(400));

        commandSegments.add(commandFactory.justDoIt(() -> robot.claw.setLeftClawClosed(true, null)));

        return commandSegments;
    }
}
