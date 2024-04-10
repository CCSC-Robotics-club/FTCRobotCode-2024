package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AprilTagCameraAndDistanceSensorAimBot;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.TeamElementFinder;

public class FormalAutoStage extends AutoStageProgram {
    public FormalAutoStage(Robot.Side side) {
        super(side);
    }

    @Override
    public void scheduleCommands(Robot robot, ModulesCommanderMarker commanderMarker, TelemetrySender telemetrySender) {
        final TeamElementFinder teamElementFinder = new TeamElementFinder(robot.hardwareMap.get(WebcamName.class, "Webcam 1"));
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, "split first(left)", new Rotation2D(Math.toRadians(90)), super.allianceSide, robot.hardwareMap);
        final AprilTagCameraAndDistanceSensorAimBot wallAimBot = new AprilTagCameraAndDistanceSensorAimBot(robot.chassis, robot.distanceSensor, robot.aprilTagCamera, null, robot.telemetrySender);
        final Runnable
                splitPreload = () -> {
                    if (super.allianceSide == Robot.Side.BLUE)
                        robot.claw.setRightClawClosed(false, null);
                    else
                        robot.claw.setLeftClawClosed(false, null);
                    },
                scorePreload = () -> {
                    if (super.allianceSide == Robot.Side.BLUE)
                        robot.claw.setLeftClawClosed(false, null);
                    else
                        robot.claw.setRightClawClosed(false, null);
                }
        ;


        super.commandSegments.add(sequentialCommandFactory.calibratePositionEstimator());
        super.commandSegments.add(sequentialCommandFactory.justDoIt(teamElementFinder::findTeamElementAndShutDown));
        super.commandSegments.add(
                new SequentialCommandSegment(
                        () -> true,
                        () -> {
                            switch (teamElementFinder.getTeamElementPosition()) {
                                case LEFT: case UNDETERMINED: return sequentialCommandFactory.getBezierCurvesFromPathFile("split first(left)").get(0);
                                case CENTER: return sequentialCommandFactory.getBezierCurvesFromPathFile("split first(mid)").get(0);
                                case RIGHT: return sequentialCommandFactory.getBezierCurvesFromPathFile("split first(right)").get(0);
                                default: throw new IllegalStateException("unknown team element position: " + teamElementFinder.getTeamElementPosition());
                            }
                        },
                        splitPreload,
                        () -> {},
                        () -> {
                            if (super.allianceSide == Robot.Side.BLUE)
                                robot.claw.setRightClawClosed(false, null);
                            else robot.claw.setLeftClawClosed(false, null);
                        },
                        robot.chassis::isCurrentTranslationalTaskComplete,
                        robot.positionEstimator::getRotation2D, () -> new Rotation2D(0)
                )
        );

        super.commandSegments.add(sequentialCommandFactory.waitFor(500));

        super.commandSegments.add(sequentialCommandFactory.moveToPoint(
                sequentialCommandFactory.getBezierCurvesFromPathFile("score second").get(0).getPositionWithLERP(1),
                () -> {
                    robot.arm.setPosition(RobotConfig.ArmConfigs.Position.SCORE, null);
                    robot.claw.setFlip(false, null);
                },
                ()->{}, ()->{}
        ));

        super.commandSegments.add(wallAimBot.stickToWall(
                teamElementFinder,
                () -> true
        ));

        super.commandSegments.add(sequentialCommandFactory.justDoIt(scorePreload));

        super.commandSegments.add(sequentialCommandFactory.waitFor(500));
        if (true)
            return;

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("move back and grab third from stack", new Rotation2D(0))
        );

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("grab fourth from stack", new Rotation2D(0))
        );

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("score third and fourth", new Rotation2D(0))
        );

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("move to stack grab fifth", new Rotation2D(0))
        );

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("grab sixth from stack", new Rotation2D(0))
        );

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("score fifth and sixth", new Rotation2D(0))
        );

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("park", new Rotation2D(0))
        );
    }
}
