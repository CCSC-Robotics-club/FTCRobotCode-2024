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

        final Vector2D stack1Position = this.allianceSide == Robot.Side.BLUE ?
                new Vector2D(new double[] {18 + 135, 20}) : new Vector2D(new double[] {0, 0});

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

        commandSegments.add(sequentialCommandFactory.followSingleCurve("score third and fourth", 1, new Rotation2D(0)));

        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> sequentialCommandFactory.getBezierCurvesFromPathFile("score third and fourth").get(0),
                () -> {
                    robot.arm.setPosition(RobotConfig.ArmConfigs.Position.SCORE, null);
                    robot.arm.setScoringHeight(0.5, null);
                },
                () -> {
                    if (!robot.arm.isArmInPosition())
                        return;

                    robot.claw.setScoringAngle(RobotConfig.ArmConfigs.flipperPositionsAccordingToActualArmAngle.getYPrediction(robot.arm.getArmDesiredPosition()), null);
                    robot.extend.setExtendPosition(RobotConfig.ArmConfigs.extendValuesAccordingToActualArmAngle.getYPrediction(robot.arm.getArmDesiredPosition()), null);
                },
                () -> {},
                () -> robot.chassis.isCurrentTranslationalTaskComplete() && robot.arm.isArmInPosition() && robot.extend.isExtendInPosition(),
                () -> robot.chassis.isVisualNavigationAvailable() && robot.arm.isArmInPosition() && robot.extend.isExtendInPosition(),
                () -> new Rotation2D(0), () -> new Rotation2D(0),
                SpeedCurves.originalSpeed, speedFactorWhenArmRaised
        ));

        commandSegments.add(wallAimBot.stickToWall(() -> true));

        commandSegments.add(sequentialCommandFactory.stayStillFor(500));
    }
}
