package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FlippableDualClaw;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;

public class AutoStageGrabbingTest extends AutoStageProgram {
    public AutoStageGrabbingTest(Robot.Side side) {
        super(side);
    }

    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, super.allianceSide, robot.hardwareMap);
        /*
        * the position at which the center of the claw is above the stack
        * */
        final Vector2D stackCenterPosition = new Vector2D(new double[] {0, -17});
        final double clawWidth = 6, grabbingDistanceToWall = 13;

        super.commandSegments.add(sequentialCommandFactory.calibratePositionEstimator());
        super.commandSegments.add(sequentialCommandFactory.moveToPointAndStop(
                stackCenterPosition.addBy(new Vector2D(new double[] {clawWidth, 25})),
                () -> {
                    robot.extend.setExtendPosition(0, null);
                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null);
                },
                () -> {}, () -> {}
        ));

        super.commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> {
                    robot.extend.setExtendPosition(RobotConfig.ExtendConfigs.grabStackValue, null);
                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.PREPARE_TO_GRAB_STACK, null);
                    robot.claw.setLeftClawClosed(false, null);
                    robot.claw.setRightClawClosed(false, null);
                },
                () -> {
                    final double wallGrabbingPositionY =
                            robot.distanceSensorBack.getSensorReading() > 30 ?
                                    stackCenterPosition.getY()
                                    : robot.positionEstimator.getCurrentPosition().getY() - robot.distanceSensorBack.getSensorReading() + grabbingDistanceToWall;
                    robot.chassis.setTranslationalTask(
                            new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                                    new Vector2D(new double[] {stackCenterPosition.getX() + clawWidth, wallGrabbingPositionY})
                            ), null);
                },
                () -> robot.claw.setFlip(FlippableDualClaw.FlipperPosition.INTAKE, null),
                robot.chassis::isCurrentTranslationalTaskComplete,
                () -> new Rotation2D(0),
                () -> new Rotation2D(0)
        ));

        super.commandSegments.add(sequentialCommandFactory.stayStillFor(600)); // wait for things to be in position

        super.commandSegments.add(sequentialCommandFactory.justDoIt(() -> robot.claw.setLeftClawClosed(true, null)));

        super.commandSegments.add(sequentialCommandFactory.stayStillFor(400));

        super.commandSegments.add(sequentialCommandFactory.justDoIt(() -> robot.claw.setFlip(FlippableDualClaw.FlipperPosition.PREPARE_TO_GRAB_STACK, null)));

        super.commandSegments.add(sequentialCommandFactory.moveToPointAndStop(new Vector2D()));
    }
}