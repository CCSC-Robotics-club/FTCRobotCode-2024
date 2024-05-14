package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.TeamElementFinder;

public class FrontFieldAutoTwoPieces extends AutoStageProgram {
    public FrontFieldAutoTwoPieces(Robot.Side side) {
        super(side);
    }

    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        final TeamElementFinder teamElementFinder = new TeamElementFinder(robot.hardwareMap.get(WebcamName.class, "Webcam 1"), super.allianceSide);
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, "split first(left)", new Rotation2D(0), super.allianceSide, robot.hardwareMap);
        final AprilTagCameraAndDistanceSensorAimBot wallAimBot = new AprilTagCameraAndDistanceSensorAimBot(robot.chassis, robot.distanceSensor, robot.aprilTagCamera, robot.arm, null, robot.telemetrySender, super.allianceSide);

        final Runnable
                splitPreload = () -> robot.claw.setRightClawClosed(false, null),
                scorePreload = () -> robot.claw.setLeftClawClosed(false, null);

        final Vector2D stack1Position = this.allianceSide == Robot.Side.BLUE ?
                new Vector2D(new double[] {18 + 135, 20}) : new Vector2D(new double[] {0, 0});

        robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null);
        robot.claw.setLeftClawClosed(true, null);
        robot.claw.setRightClawClosed(true, null);


        super.commandSegments.add(sequentialCommandFactory.calibratePositionEstimator());
        // super.commandSegments.add(teamElementFinder.findTeamElementAndShutDown(5000));
        teamElementFinder.teamElementPosition = TeamElementFinder.TeamElementPosition.RIGHT;

        // TODO split preload positions measuring
        super.commandSegments.add(
                new SequentialCommandSegment(
                        () -> true,
                        () -> {
                            final Vector2D splitFirstPosition;
                            switch (teamElementFinder.teamElementPosition) {
                                case LEFT: case UNDETERMINED: {
                                    splitFirstPosition = this.allianceSide == Robot.Side.BLUE ?
                                            new Vector2D(new double[] {90, 270}) : new Vector2D(new double[] {0, 0});
                                    break;
                                }
                                case CENTER: {
                                    splitFirstPosition = this.allianceSide == Robot.Side.BLUE ?
                                            new Vector2D(new double[] {120, 246}) : new Vector2D(new double[] {0, 0});
                                    break;
                                }
                                case RIGHT: {
                                    splitFirstPosition = this.allianceSide == Robot.Side.BLUE ?
                                            new Vector2D(new double[] {92, 210}) : new Vector2D(new double[] {0, 0});
                                    break;
                                }
                                default:
                                    throw new IllegalStateException("unknown team element position: " + teamElementFinder.teamElementPosition);
                            }

                            return new BezierCurve(
                                    sequentialCommandFactory.getRobotStartingPosition("split first(left)"),
                                    splitFirstPosition.addBy(new Vector2D(new double[] {0, 10})),
                                    splitFirstPosition
                            );
                            },
                        () -> {},
                        () -> {
                            robot.claw.setFlip(FlippableDualClaw.FlipperPosition.PREPARE_TO_GRAB_STACK, null);
                            if (robot.chassis.isCurrentRotationalTaskRoughlyComplete())
                                robot.extend.setExtendPosition(RobotConfig.ExtendConfigs.intakeValue, null);
                        },
                        splitPreload,
                        robot.chassis::isCurrentTranslationalTaskComplete,
                        () -> new Rotation2D(0), () -> new Rotation2D(0)
                )
        );
        super.commandSegments.add(sequentialCommandFactory.stayStillFor(300));

        super.commandSegments.add(sequentialCommandFactory.justDoIt(() -> {
            robot.claw.setRightClawClosed(true, null);
            robot.extend.setExtendPosition(0, null);
            robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null);
        }));
        super.commandSegments.add(sequentialCommandFactory.stayStillFor(300));
        super.commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(robot.positionEstimator.getCurrentPosition(), sequentialCommandFactory.getBezierCurvesFromPathFile("score second").get(0).getPositionWithLERP(1)),
                () -> {
                    robot.arm.setPosition(RobotConfig.ArmConfigs.Position.SCORE, null);
                    robot.arm.setScoringHeight(0, null);
                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.SCORE, null);
                    robot.claw.setScoringAngle(1, null);
                    robot.extend.setExtendPosition(0, null);
                },
                () -> {},
                () -> {
                    robot.extend.setExtendPosition(500, null);
                    robot.claw.setScoringAngle(RobotConfig.ArmConfigs.flipperPositionsAccordingToActualArmAngle.getYPrediction(robot.arm.getArmDesiredPosition()) ,null);
                    },
                () -> robot.chassis.isCurrentTranslationalTaskComplete() && robot.arm.isArmInPosition(),
                () -> new Rotation2D(0), () -> new Rotation2D(0),
                SpeedCurves.originalSpeed, 0.5
        ));

        super.commandSegments.add(wallAimBot.stickToWall(
                teamElementFinder,
                robot.extend::isExtendInPosition
        ));

        super.commandSegments.add(sequentialCommandFactory.stayStillFor(500));

        super.commandSegments.add(sequentialCommandFactory.justDoIt(scorePreload));

        super.commandSegments.add(sequentialCommandFactory.stayStillFor(500));
    }
}
