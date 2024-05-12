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
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.TeamElementFinder;

import java.util.PrimitiveIterator;

public class FrontFieldAutoFourPieces extends AutoStageProgram {
    public FrontFieldAutoFourPieces(Robot.Side side) {
        super(side);
    }

    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        final TeamElementFinder teamElementFinder = new TeamElementFinder(robot.hardwareMap.get(WebcamName.class, "Webcam 1"));
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, "split first(left)", new Rotation2D(Math.toRadians(90)), super.allianceSide, robot.hardwareMap);
        final AprilTagCameraAndDistanceSensorAimBot wallAimBot = new AprilTagCameraAndDistanceSensorAimBot(robot.chassis, robot.distanceSensor, robot.aprilTagCamera, robot.arm, null, robot.telemetrySender);
        final double speedConstrainWhenArmRaised = 0.6;


        final Runnable
                splitPreload = () -> robot.claw.setLeftClawClosed(false, null),
                scorePreload = () -> robot.claw.setRightClawClosed(false, null);
        final Vector2D stackCenterPositionDefault = new Vector2D(new double[] {18 + 135, 20});

        robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null);
        robot.claw.setLeftClawClosed(true, null);
        robot.claw.setRightClawClosed(true, null);


        super.commandSegments.add(sequentialCommandFactory.calibratePositionEstimator());
        super.commandSegments.add(teamElementFinder.findTeamElementAndShutDown(5000));

        // TODO split preload positions measuring
        super.commandSegments.add(
                new SequentialCommandSegment(
                        () -> true,
                        () -> {
                            switch (teamElementFinder.getTeamElementPosition()) {
                                case LEFT: case UNDETERMINED:
                                    return sequentialCommandFactory.getBezierCurvesFromPathFile("split first(left)").get(0);
                                case CENTER:
                                    return sequentialCommandFactory.getBezierCurvesFromPathFile("split first(center)").get(0);
                                case RIGHT:
                                    return sequentialCommandFactory.getBezierCurvesFromPathFile("split first(right)").get(0);
                                default:
                                    throw new IllegalStateException("unknown team element position: " + teamElementFinder.getTeamElementPosition());
                            }
                            },
                        () -> {
                            robot.extend.setExtendPosition(RobotConfig.ExtendConfigs.intakeValue, null);
                            robot.claw.setFlip(FlippableDualClaw.FlipperPosition.PREPARE_TO_GRAB_STACK, null);
                        },
                        () -> {},
                        () -> robot.claw.setFlip(FlippableDualClaw.FlipperPosition.INTAKE, null),
                        robot.chassis::isCurrentTranslationalTaskComplete,
                        robot.positionEstimator::getRotation2D, () -> new Rotation2D(0)
                )
        );

        super.commandSegments.add(sequentialCommandFactory.waitFor(500));
        super.commandSegments.add(sequentialCommandFactory.justDoIt(splitPreload));
        super.commandSegments.add(sequentialCommandFactory.waitFor(300));

        super.commandSegments.add(sequentialCommandFactory.justDoIt(() ->
                robot.arm.setPosition(RobotConfig.ArmConfigs.Position.SCORE, null)));

        super.commandSegments.add(sequentialCommandFactory.waitFor(200));

        super.commandSegments.add(sequentialCommandFactory.moveToPoint(
                sequentialCommandFactory.getBezierCurvesFromPathFile("score second").get(0).getPositionWithLERP(1),
                () -> {
                    robot.arm.setScoringHeight(0, null);
                    robot.claw.setLeftClawClosed(true, null);
                    robot.claw.setRightClawClosed(true, null);
                    robot.arm.setPosition(RobotConfig.ArmConfigs.Position.SCORE, null);
                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null);
                },
                () -> {
                }, () -> {
                }
        ));

        super.commandSegments.add(wallAimBot.stickToWall(
                teamElementFinder,
                robot.arm::isArmInPosition
        ));

        super.commandSegments.add(sequentialCommandFactory.waitFor(300));

        super.commandSegments.add(sequentialCommandFactory.justDoIt(scorePreload));

        super.commandSegments.add(sequentialCommandFactory.waitFor(500));

    }
}
