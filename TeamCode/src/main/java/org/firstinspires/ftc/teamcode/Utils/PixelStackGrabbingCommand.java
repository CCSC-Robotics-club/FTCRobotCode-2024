package org.firstinspires.ftc.teamcode.Utils;

import static org.firstinspires.ftc.teamcode.RobotConfig.VisualNavigationConfigs.*;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FlippableDualClaw;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedSensor;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.SpeedCurves;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class PixelStackGrabbingCommand {
    public static List<SequentialCommandSegment> getCommandSegmentSegments(Robot robot, SequentialCommandFactory commandFactory, Vector2D stackCenterPositionDefault) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final double clawWidth = 6.5, grabbingDistanceToWall = 13;

        /* step 1: line up the lefter claw with the stack */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> robot.claw.setFlip(FlippableDualClaw.FlipperPosition.PREPARE_TO_GRAB_STACK, null),
                () -> {
                    final double wallPositionY = robot.distanceSensorBack.getSensorReading() < 50 ?
                            robot.positionEstimator.getCurrentPosition().getY() - robot.distanceSensorBack.getSensorReading()
                            : stackCenterPositionDefault.getY();
                    robot.chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                            new Vector2D(new double[] {stackCenterPositionDefault.getX() + clawWidth, wallPositionY + 20})), null);
                },
                () -> {
                    robot.claw.setLeftClawClosed(false, null);
                    robot.claw.setRightClawClosed(false, null);
                    robot.extend.setExtendPosition(RobotConfig.ExtendConfigs.grabStackValue, null);
                },
                robot.chassis::isCurrentTranslationalTaskComplete,
                () -> new Rotation2D(0),
                () -> new Rotation2D(0)
        ));

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
                            new Vector2D(new double[] {stackCenterPositionDefault.getX() + clawWidth, wallPositionY + grabbingDistanceToWall})), null);
                },
                () -> {},
                robot.chassis::isCurrentTranslationalTaskComplete,
                () -> new Rotation2D(0),
                () -> new Rotation2D(0)
        ));

        final Vector2D robotPositionGrabbingRightClaw = new Vector2D();
        commandSegments.add(commandFactory.justDoIt(
                () -> robotPositionGrabbingRightClaw.update(new double[] {stackCenterPositionDefault.getX() - clawWidth, robot.positionEstimator.getCurrentPosition().getY()})
        ));

        commandSegments.add(commandFactory.justDoIt(() -> robot.claw.setFlip(FlippableDualClaw.FlipperPosition.INTAKE, null)));

        commandSegments.add(commandFactory.waitFor(500));

        commandSegments.add(commandFactory.justDoIt(() -> robot.claw.setLeftClawClosed(true, null)));

        commandSegments.add(commandFactory.waitFor(500));

        commandSegments.add(commandFactory.justDoIt(() -> robot.claw.setFlip(FlippableDualClaw.FlipperPosition.PREPARE_TO_GRAB_STACK, null)));

        commandSegments.add(commandFactory.moveToPoint(new Vector2D(new double[] {stackCenterPositionDefault.getX() + clawWidth, 40})));

        commandSegments.add(commandFactory.moveToPointAndStop(new Vector2D(new double[] {stackCenterPositionDefault.getX() - clawWidth, 40})));

        commandSegments.add(commandFactory.moveToPointAndStop(robotPositionGrabbingRightClaw));

        commandSegments.add(commandFactory.justDoIt(() -> robot.claw.setFlip(FlippableDualClaw.FlipperPosition.INTAKE, null)));

        commandSegments.add(commandFactory.waitFor(500));

        commandSegments.add(commandFactory.justDoIt(() -> robot.claw.setRightClawClosed(true, null)));

        commandSegments.add(commandFactory.waitFor(400));

        commandSegments.add(commandFactory.justDoIt(() -> robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null)));

        commandSegments.add(commandFactory.waitFor(400));

        return commandSegments;
    }

    public static List<SequentialCommandSegment> getCommandSegmentSegmentsWithColorSensor(Robot robot, ThreadedSensor markDetector, SequentialCommandFactory commandFactory, Vector2D stackCenterPositionDefault) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final Vector2D scanningStartPosition = stackCenterPositionDefault.addBy(new Vector2D(new double[] {20, scanningDistanceToWall}));

        final double startingPositionX = scanningStartPosition.getX();
        /* step 1: drive a smooth curve to the starting point of the spike-mark detecting process */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> {
                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null);
                    robot.claw.setRightClawClosed(true, null);
                    robot.claw.setLeftClawClosed(true, null);
                    robot.extend.setExtendPosition(0, null);
                    robot.arm.setPosition(RobotConfig.ArmConfigs.Position.INTAKE, null);
                },
                () -> {
                    final double wallPositionY = robot.distanceSensorBack.getSensorReading() < 50 ?
                            robot.positionEstimator.getCurrentPosition().getY() - robot.distanceSensorBack.getSensorReading()
                            : stackCenterPositionDefault.getY();
                    robot.chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                            new Vector2D(new double[] {startingPositionX, wallPositionY + scanningDistanceToWall})), null);
                },
                () -> {
                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.PREPARE_TO_GRAB_STACK, null);
                    robot.extend.setExtendPosition(RobotConfig.ExtendConfigs.grabStackValue, null);
                },
                robot.chassis::isCurrentTranslationalTaskComplete,
                () -> new Rotation2D(0),
                () -> new Rotation2D(0)
        ));



        /* step 2: move along a horizontal path with distance to wall 17cm (using distance sensor) and find the spike mark */
        final double[] actualStackCenterPositionX = new double[] {stackCenterPositionDefault.getX(), -1, -1};
        final Vector2D scanningStartingPosition = new Vector2D(new double[] {startingPositionX, stackCenterPositionDefault.getY()+ scanningDistanceToWall}),
                scanningEndingPosition = new Vector2D(new double[] {stackCenterPositionDefault.getX(), stackCenterPositionDefault.getY() + scanningDistanceToWall});
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> {
                    robot.chassis.setLowSpeedModeEnabled(true);
                    final double wallPositionY = robot.distanceSensorBack.getSensorReading() < 50 ?
                            robot.positionEstimator.getCurrentPosition().getY() - robot.distanceSensorBack.getSensorReading()
                            : stackCenterPositionDefault.getY();
                    return new BezierCurve(
                            new Vector2D(new double[] {startingPositionX, wallPositionY + scanningDistanceToWall}),
                            new Vector2D(new double[] {stackCenterPositionDefault.getX() - 5, wallPositionY + scanningDistanceToWall})
                    );
                },
                () -> {},
                () -> {
                    final double wallPositionY = robot.distanceSensorBack.getSensorReading() < 60 ?
                        robot.positionEstimator.getCurrentPosition().getY() - robot.distanceSensorBack.getSensorReading()
                        : stackCenterPositionDefault.getY();
                    scanningStartingPosition.update(new double[] {
                            scanningStartingPosition.getX(), wallPositionY + scanningDistanceToWall
                    });
                    scanningEndingPosition.update(new double[] {
                            scanningEndingPosition.getX(), wallPositionY + scanningDistanceToWall
                    });

                    if (markDetector.getSensorReading() >= colorSensorThreshold && actualStackCenterPositionX[1] == -1)
                        actualStackCenterPositionX[1] = robot.positionEstimator.getCurrentPosition().getX() + colorSensorPositionOnRobot;
                    if (markDetector.getSensorReading() < colorSensorThreshold && actualStackCenterPositionX[1] != -1 && actualStackCenterPositionX[2] == -1)
                        actualStackCenterPositionX[2] = robot.positionEstimator.getCurrentPosition().getX() + colorSensorPositionOnRobot;
                },
                () -> {
                    robot.chassis.setLowSpeedModeEnabled(false);
                    if (actualStackCenterPositionX[1] != -1 && actualStackCenterPositionX[2] != -1)
                        actualStackCenterPositionX[0] = (actualStackCenterPositionX[1] + actualStackCenterPositionX[2])/2;
                    else throw new IllegalStateException("spike mark unseen");
                },
                robot.chassis::isCurrentTranslationalTaskComplete,
                () -> actualStackCenterPositionX[1] != -1 && actualStackCenterPositionX[2] != -1,
                () -> new Rotation2D(0),
                () -> new Rotation2D(0),
                SpeedCurves.originalSpeed,
                0.1
        ));

//        /* step 3: line up the lefter claw with the stack */
//        commandSegments.add(new SequentialCommandSegment(
//                () -> true,
//                () -> null,
//                () -> {
//                    robot.claw.setLeftClawClosed(false, null);
//                    robot.claw.setRightClawClosed(false, null);
//                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.PREPARE_TO_GRAB_STACK, null);
//                },
//                () -> {
//                    final double wallPositionY = robot.distanceSensorBack.getSensorReading() < 50 ?
//                            robot.positionEstimator.getCurrentPosition().getY() - robot.distanceSensorBack.getSensorReading()
//                            : stackCenterPositionDefault.getY();
//                    robot.chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
//                            new Vector2D(new double[] {actualStackCenterPositionX[0] + clawWidth, wallPositionY + scanningDistanceToWall})), null);
//                },
//                () -> {},
//                robot.chassis::isCurrentTranslationalTaskComplete,
//                () -> new Rotation2D(0),
//                () -> new Rotation2D(0)
//        ));

        commandSegments.add(commandFactory.justDoIt(() -> {
            robot.claw.setLeftClawClosed(false, null);
            robot.claw.setRightClawClosed(false, null);
            robot.claw.setFlip(FlippableDualClaw.FlipperPosition.PREPARE_TO_GRAB_STACK, null);
        }));

        /* step 3: drive back to the grabbing distance */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> robot.extend.setExtendPosition(RobotConfig.ExtendConfigs.grabStackValue, null),
                () -> {
                    final double wallPositionY = robot.distanceSensorBack.getSensorReading() < 50 ?
                            robot.positionEstimator.getCurrentPosition().getY() - robot.distanceSensorBack.getSensorReading()
                            : stackCenterPositionDefault.getY();
                    robot.chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                            new Vector2D(new double[] {actualStackCenterPositionX[0] + clawWidth, wallPositionY + grabbingDistanceToWall})), null);
                },
                () -> {},
                robot.chassis::isCurrentTranslationalTaskComplete,
                () -> new Rotation2D(0),
                () -> new Rotation2D(0)
        ));

        final Vector2D robotPositionGrabbingRightClaw = new Vector2D();
        commandSegments.add(commandFactory.justDoIt(
                () -> robotPositionGrabbingRightClaw.update(new double[] {actualStackCenterPositionX[0] - clawWidth, robot.positionEstimator.getCurrentPosition().getY()})
        ));

        commandSegments.add(commandFactory.justDoIt(() -> robot.claw.setFlip(FlippableDualClaw.FlipperPosition.INTAKE, null)));

        commandSegments.add(commandFactory.waitFor(500));

        commandSegments.add(commandFactory.justDoIt(() -> robot.claw.setLeftClawClosed(true, null)));

        commandSegments.add(commandFactory.waitFor(500));

        commandSegments.add(commandFactory.justDoIt(() -> robot.claw.setFlip(FlippableDualClaw.FlipperPosition.PREPARE_TO_GRAB_STACK, null)));

        commandSegments.add(commandFactory.moveToPoint(new Vector2D(new double[] {actualStackCenterPositionX[0] + clawWidth, 40})));

        commandSegments.add(commandFactory.moveToPointAndStop(new Vector2D(new double[] {actualStackCenterPositionX[0] - clawWidth, 40})));

        commandSegments.add(commandFactory.moveToPointAndStop(robotPositionGrabbingRightClaw));

        commandSegments.add(commandFactory.justDoIt(() -> robot.claw.setFlip(FlippableDualClaw.FlipperPosition.INTAKE, null)));

        commandSegments.add(commandFactory.waitFor(500));

        commandSegments.add(commandFactory.justDoIt(() -> robot.claw.setRightClawClosed(true, null)));

        commandSegments.add(commandFactory.waitFor(400));

        commandSegments.add(commandFactory.justDoIt(() -> robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null)));

        commandSegments.add(commandFactory.waitFor(400));

        return commandSegments;
    }
}
