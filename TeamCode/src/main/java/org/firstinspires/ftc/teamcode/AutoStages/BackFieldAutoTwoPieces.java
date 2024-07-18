package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Modules.FlippableDualClaw;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AprilTagCameraAndDistanceSensorAimBot;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.TeamElementFinderColor;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.SpeedCurves;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;

import java.util.concurrent.atomic.AtomicReference;

public class BackFieldAutoTwoPieces extends AutoStageProgram {
    protected BackFieldAutoTwoPieces(Robot.Side side) {
        super(side);
    }

    private static final double sleepSeconds = 10;
    private static final Vector2D
            BLUE_LEFT_SPIKE = new Vector2D(new double[] {0, 0}),
            BLUE_CENTER_SPIKE = new Vector2D(new double[] {0, 0}),
            BLUE_RIGHT_SPIKE = new Vector2D(new double[] {0, 0}),

    RED_LEFT_SPIKE = new Vector2D(new double[] {0, 0}),
            RED_CENTER_SPIKE = new Vector2D(new double[] {0, 0}),
            RED_RIGHT_SPIKE = new Vector2D(new double[] {0, 0});

    private TeamElementFinderColor teamElementFinderColor;
    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        this.teamElementFinderColor = new TeamElementFinderColor(
                robot.hardwareMap,
                robot.hardwareMap.get(WebcamName.class, allianceSide == Robot.Side.BLUE ? "Right Cam" : "Left Cam"),
                robot.gamepad1,
                robot.telemetry,
                allianceSide
        );
        final AtomicReference<RobotConfig.TeamElementPosition> teamElementPositionReference = new AtomicReference<>(RobotConfig.TeamElementPosition.UNDETERMINED);

        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, "split left back stage", new Rotation2D(Math.PI), super.allianceSide, robot.hardwareMap);
        final AprilTagCameraAndDistanceSensorAimBot wallAimBot = new AprilTagCameraAndDistanceSensorAimBot(robot.chassis, robot.distanceSensor, robot.aprilTagCamera, null, robot.telemetrySender, super.allianceSide);

        final Runnable
                splitPreload = () -> robot.claw.setRightClawClosed(false, null),
                scorePreload = () -> robot.claw.setLeftClawClosed(false, null);

        robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null);
        robot.claw.setLeftClawClosed(true, null);
        robot.claw.setRightClawClosed(true, null);


        super.commandSegments.add(commandFactory.calibratePositionEstimator());

        final AtomicReference<Vector2D> splitFirstPositionReference = new AtomicReference<>(
                this.allianceSide == Robot.Side.BLUE ? BLUE_LEFT_SPIKE : RED_LEFT_SPIKE
        );
        super.commandSegments.add(commandFactory.justDoIt(() ->
        {
            teamElementPositionReference.set(teamElementFinderColor.getBestResult());
            switch (teamElementFinderColor.getBestResult()) {
                case LEFT: case UNDETERMINED: {
                    splitFirstPositionReference.set(
                            this.allianceSide == Robot.Side.BLUE ?
                                    BLUE_LEFT_SPIKE : RED_LEFT_SPIKE
                    );
                    break;
                }
                case CENTER: {
                    splitFirstPositionReference.set(
                            this.allianceSide == Robot.Side.BLUE ?
                                    BLUE_CENTER_SPIKE : RED_CENTER_SPIKE
                    );
                    break;
                }
                case RIGHT: {
                    splitFirstPositionReference.set(
                            this.allianceSide == Robot.Side.BLUE ?
                                    BLUE_RIGHT_SPIKE : RED_RIGHT_SPIKE
                    );
                    break;
                }
                default:
                    throw new IllegalStateException("unknown team element position: " + teamElementFinderColor.getBestResult());
            }
        }));
        super.commandSegments.add(
                new SequentialCommandSegment(
                        () -> true,
                        () -> new BezierCurve(
                                commandFactory.getRobotStartingPosition("split left back stage"),
                                splitFirstPositionReference.get().addBy(new Vector2D(new double[] {0, -20})),
                                splitFirstPositionReference.get()
                        ),
                        () -> {},
                        () -> {
                            if (!robot.chassis.isCurrentTranslationalTaskRoughlyComplete())
                                return;
                            robot.claw.setFlip(FlippableDualClaw.FlipperPosition.PREPARE_TO_GRAB_STACK, null);
                            robot.extend.setExtendPosition(RobotConfig.ExtendConfigs.intakeValue, null);
                        },
                        splitPreload,
                        () -> Vector2D.displacementToTarget(robot.positionEstimator.getCurrentPosition(), splitFirstPositionReference.get()).getMagnitude() < 5,
                        () -> new Rotation2D(0), () -> new Rotation2D(0)
                )
        );
        super.commandSegments.add(commandFactory.stayStillFor(300));

        super.commandSegments.add(commandFactory.followSingleCurve(
                "move to scoring board back stage", 0,
                new Rotation2D(Math.toRadians(-90))
        ));

        super.commandSegments.add(commandFactory.stayStillForSeconds(sleepSeconds));

        super.commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> commandFactory.getBezierCurvesFromPathFile("move to scoring board back stage").get(1),
                () -> {
                    robot.claw.setRightClawClosed(true, null);
                    robot.extend.setExtendPosition(0, null);
                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null);

                    robot.arm.setPosition(RobotConfig.ArmConfigs.Position.PREPARE_TO_SCORE, null);
                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.SCORE, null);
                    robot.claw.setScoringAngle(1, null);
                    robot.extend.setExtendPosition(0, null);
                },
                () -> {
                    robot.arm.setScoringHeight(RobotConfig.ArmConfigs.autoStageArmScoringHeight, null);
                    if (robot.arm.isArmInPosition())
                        robot.arm.setPosition(RobotConfig.ArmConfigs.Position.SCORE, null);
                },
                () -> {
                    robot.extend.setExtendPosition(RobotConfig.ArmConfigs.autoStageScoringExtendPosition, null);
                    robot.claw.setScoringAngle(RobotConfig.ArmConfigs.autoStageScoringServoPosition, null);
                },
                () -> robot.chassis.isCurrentTranslationalTaskComplete()
                        && robot.arm.getArmDesiredPosition() == RobotConfig.ArmConfigs.Position.SCORE
                        && robot.arm.isArmInPosition(),
                () -> new Rotation2D(Math.toRadians(-90)), () -> new Rotation2D(0),
                SpeedCurves.originalSpeed, 0.5
        ));

        super.commandSegments.add(wallAimBot.stickToWall(
                teamElementPositionReference::get,
                robot.extend::isExtendInPosition
        ));

        super.commandSegments.add(commandFactory.stayStillFor(300));

        super.commandSegments.add(commandFactory.justDoIt(scorePreload));

        super.commandSegments.add(commandFactory.stayStillFor(500));

        super.commandSegments.add(commandFactory.followSingleCurve(
                "park back stage", 0,
                new Rotation2D(0),
                () -> {
                    robot.claw.setLeftClawClosed(true, null);
                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null);
                    robot.extend.setExtendPosition(0, null);
                    robot.arm.setPosition(RobotConfig.ArmConfigs.Position.INTAKE, null);
                },
                () -> {},
                () -> {}
        ));

        super.commandSegments.add(commandFactory.stayStillFor(2000));
    }

    @Override
    public void beforeStartPeriodic() {
        teamElementFinderColor.beforeStartPeriodic();
    }
}
