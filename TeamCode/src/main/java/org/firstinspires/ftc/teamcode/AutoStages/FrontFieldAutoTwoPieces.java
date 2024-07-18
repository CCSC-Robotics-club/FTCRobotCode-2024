package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.teamcode.Modules.FlippableDualClaw;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AprilTagCameraAndDistanceSensorAimBot;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.SpeedCurves;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.TeamElementFinderTensorflow;

public class FrontFieldAutoTwoPieces extends AutoStageProgram {
    public FrontFieldAutoTwoPieces(Robot.Side side) {
        super(side);
    }

    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        final TeamElementFinderTensorflow teamElementFinder = new TeamElementFinderTensorflow(robot.hardwareMap, super.allianceSide);
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, "split first(left)", new Rotation2D(0), super.allianceSide, robot.hardwareMap);
        final AprilTagCameraAndDistanceSensorAimBot wallAimBot = new AprilTagCameraAndDistanceSensorAimBot(robot.chassis, robot.distanceSensor, robot.aprilTagCamera, robot.arm, null, robot.telemetrySender, super.allianceSide);

        final Runnable
                splitPreload = () -> robot.claw.setRightClawClosed(false, null),
                scorePreload = () -> robot.claw.setLeftClawClosed(false, null);

        robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null);
        robot.claw.setLeftClawClosed(true, null);
        robot.claw.setRightClawClosed(true, null);


        super.commandSegments.add(sequentialCommandFactory.calibratePositionEstimator());
        // teamElementFinder.teamElementPosition = TeamElementFinder.TeamElementPosition.LEFT;
        super.commandSegments.add(teamElementFinder.findTeamElementAndShutDown(2000));

        final Vector2D[] splitFirstPosition = new Vector2D[1];
        super.commandSegments.add(
                new SequentialCommandSegment(
                        () -> true,
                        () -> {
                            switch (teamElementFinder.teamElementPosition) {
                                case LEFT: case UNDETERMINED: {
                                    splitFirstPosition[0] = this.allianceSide == Robot.Side.BLUE ?
                                            new Vector2D(new double[] {92, 275}) : new Vector2D(new double[] {248, 218});
                                    break;
                                }
                                case CENTER: {
                                    splitFirstPosition[0] = this.allianceSide == Robot.Side.BLUE ?
                                            new Vector2D(new double[] {121, 252}) : new Vector2D(new double[] {240, 252});
                                    break;
                                }
                                case RIGHT: {
                                    splitFirstPosition[0] = this.allianceSide == Robot.Side.BLUE ?
                                            new Vector2D(new double[] {92, 221}) : new Vector2D(new double[] {253, 271});
                                    break;
                                }
                                default:
                                    throw new IllegalStateException("unknown team element position: " + teamElementFinder.teamElementPosition);
                            }

                            return new BezierCurve(
                                    sequentialCommandFactory.getRobotStartingPosition("split first(left)"),
                                    splitFirstPosition[0].addBy(new Vector2D(new double[] {0, 15})),
                                    splitFirstPosition[0]
                            );
                            },
                        () -> {},
                        () -> {
                            if (!robot.chassis.isCurrentTranslationalTaskRoughlyComplete())
                                return;
                            robot.claw.setFlip(FlippableDualClaw.FlipperPosition.PREPARE_TO_GRAB_STACK, null);
                            robot.extend.setExtendPosition(RobotConfig.ExtendConfigs.intakeValue, null);
                        },
                        splitPreload,
                        () -> Vector2D.displacementToTarget(robot.positionEstimator.getCurrentPosition(), splitFirstPosition[0]).getMagnitude() < 5,
                        () -> new Rotation2D(0), () -> new Rotation2D(0)
                )
        );
        super.commandSegments.add(sequentialCommandFactory.stayStillFor(300));

        super.commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(robot.positionEstimator.getCurrentPosition(), sequentialCommandFactory.getBezierCurvesFromPathFile("score second").get(0).getPositionWithLERP(1)),
                () -> {
                    robot.claw.setRightClawClosed(true, null);
                    robot.extend.setExtendPosition(0, null);
                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null);

                    robot.arm.setPosition(RobotConfig.ArmConfigs.Position.PREPARE_TO_SCORE, null);
                    robot.arm.setScoringHeight(0, null);
                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.SCORE, null);
                    robot.claw.setScoringAngle(1, null);
                    robot.extend.setExtendPosition(0, null);
                },
                () -> {
                    if (robot.arm.isArmInPosition())
                        robot.arm.setPosition(RobotConfig.ArmConfigs.Position.SCORE, null);
                },
                () -> {
                    robot.extend.setExtendPosition(100, null);
                    robot.claw.setScoringAngle(RobotConfig.ArmConfigs.flipperPositionsAccordingToScoringHeight.getYPrediction(0), null);
                    },
                () -> robot.chassis.isCurrentTranslationalTaskComplete() && robot.arm.getArmDesiredPosition() == RobotConfig.ArmConfigs.Position.SCORE && robot.arm.isArmInPosition(),
                () -> new Rotation2D(0), () -> new Rotation2D(0),
                SpeedCurves.originalSpeed, 0.5
        ));

        super.commandSegments.add(wallAimBot.stickToWall(
                teamElementFinder,
                14,
                robot.extend::isExtendInPosition
        ));

        super.commandSegments.add(sequentialCommandFactory.stayStillFor(300));

        super.commandSegments.add(sequentialCommandFactory.justDoIt(scorePreload));

        super.commandSegments.add(sequentialCommandFactory.stayStillFor(300));
    }

    @Override
    public void beforeStartPeriodic() {

    }
}
