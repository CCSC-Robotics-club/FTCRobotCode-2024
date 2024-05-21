package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.teamcode.Modules.FlippableDualClaw;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AprilTagCameraAndDistanceSensorAimBot;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.SpeedCurves;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.PixelStackGrabbingCommand;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;

public class FrontFieldAutoFourPieces extends AutoStageProgram {
    public FrontFieldAutoFourPieces(Robot.Side side) {
        super(side);
    }

    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        final AprilTagCameraAndDistanceSensorAimBot wallAimBot = new AprilTagCameraAndDistanceSensorAimBot(robot.chassis, robot.distanceSensor, robot.aprilTagCamera, robot.arm, null, robot.telemetrySender, super.allianceSide);
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, "split first(left)", new Rotation2D(0), super.allianceSide, robot.hardwareMap);
        final AutoStageProgram firstTwoPieces = new FrontFieldAutoTwoPieces(allianceSide);
        final double speedFactorWhenArmRaised = 0.6;

        final Vector2D stack1Position = new Vector2D(new double[] {
                this.allianceSide == Robot.Side.BLUE ? 155 : 220,
                20
        });

        firstTwoPieces.scheduleCommands(robot, telemetrySender);
        commandSegments.addAll(firstTwoPieces.commandSegments);

        commandSegments.add(sequentialCommandFactory.followSingleCurve(
                "move back and grab third from stack",
                0,
                new Rotation2D(0),
                () -> {
                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null);
                    robot.extend.setExtendPosition(0, null);
                },
                () -> {
                    if (!robot.extend.isExtendInPosition())
                        return;

                    robot.arm.setPosition(RobotConfig.ArmConfigs.Position.INTAKE, null);
                    robot.claw.setLeftClawClosed(true, null);
                    robot.claw.setRightClawClosed(true, null);
                },
                () -> {},
                SpeedCurves.originalSpeed,
                speedFactorWhenArmRaised
        ));
        commandSegments.add(sequentialCommandFactory.followSingleCurve("move back and grab third from stack", 1, new Rotation2D(0)));
        commandSegments.addAll(PixelStackGrabbingCommand.getCommandSegmentSegmentsWithColorSensor(robot, robot.spikeMarkDetectionSensor, sequentialCommandFactory, stack1Position));

        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> sequentialCommandFactory.getBezierCurvesFromPathFile("score third and fourth").get(0),
                () -> {},
                () -> {},
                () -> robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null),
                robot.chassis::isCurrentTranslationalTaskRoughlyComplete,
                () -> new Rotation2D(0), () -> new Rotation2D(0)
        ));

        final double scoringHeight = 0.5;
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> sequentialCommandFactory.getBezierCurvesFromPathFile("score third and fourth").get(1),
                () -> robot.arm.setPosition(RobotConfig.ArmConfigs.Position.PREPARE_TO_SCORE, null),
                () -> {
                    if (robot.arm.isArmInPosition())
                        robot.arm.setPosition(RobotConfig.ArmConfigs.Position.SCORE, null);
                    robot.arm.setScoringHeight(scoringHeight, null);
                },
                () -> {
                    robot.claw.setScoringAngle(RobotConfig.ArmConfigs.flipperPositionsAccordingToScoringHeight.getYPrediction(scoringHeight), null);
                    robot.extend.setExtendPosition(750, null);
                },
                () -> robot.chassis.isCurrentTranslationalTaskComplete() && robot.arm.getArmDesiredPosition() == RobotConfig.ArmConfigs.Position.SCORE && robot.arm.isArmInPosition() && robot.extend.isExtendInPosition(),
                () -> robot.chassis.isVisualNavigationAvailable() && robot.arm.isArmInPosition() && robot.extend.isExtendInPosition(),
                () -> new Rotation2D(0), () -> new Rotation2D(0),
                SpeedCurves.originalSpeed, speedFactorWhenArmRaised
        ));

        commandSegments.add(wallAimBot.stickToWall(
                new Vector2D(new double[] {0, -14}),
                () -> true
        ));

        commandSegments.add(sequentialCommandFactory.stayStillFor(300));

        commandSegments.add(sequentialCommandFactory.justDoIt(() -> {
            robot.claw.setLeftClawClosed(false, null);
            robot.claw.setRightClawClosed(false, null);
        }));

        commandSegments.add(sequentialCommandFactory.stayStillFor(300));
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> sequentialCommandFactory.getBezierCurvesFromPathFile("park").get(0),
                () -> {
                    robot.arm.setPosition(RobotConfig.ArmConfigs.Position.INTAKE, null);
                    robot.extend.setExtendPosition(0, null);
                    robot.claw.setRightClawClosed(true, null);
                    robot.claw.setLeftClawClosed(true, null);
                },() -> {}, () -> {},
                () -> robot.arm.isArmInPosition(),
                () -> new Rotation2D(0), () -> new Rotation2D(0)
        ));
    }
}
