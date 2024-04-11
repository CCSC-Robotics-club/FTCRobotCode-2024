package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AprilTagCameraAndDistanceSensorAimBot;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.SpeedCurves;
import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.TeamElementFinder;

public class FrontFieldAutoTwoPieces extends AutoStageProgram {
    public FrontFieldAutoTwoPieces(Robot.Side side) {
        super(side);
    }

    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        final TeamElementFinder teamElementFinder = new TeamElementFinder(robot.hardwareMap.get(WebcamName.class, "Webcam 1"));
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, "split first(left)", new Rotation2D(Math.toRadians(90)), super.allianceSide, robot.hardwareMap);
        final AprilTagCameraAndDistanceSensorAimBot wallAimBot = new AprilTagCameraAndDistanceSensorAimBot(robot.chassis, robot.distanceSensor, robot.aprilTagCamera, robot.arm, null, robot.telemetrySender);
        final double speedConstrainWhenArmRaised = 0.5;
        final Runnable
                splitPreload = () -> {
                    if (super.allianceSide == Robot.Side.BLUE)
                        robot.claw.setRightClawClosed(false, null);
                    else
                        robot.claw.setLeftClawClosed(false, null);
                    },
                scorePreload = () -> {
                    robot.claw.setLeftClawClosed(false, null);
                    robot.claw.setRightClawClosed(false, null);
                };
        final SequentialCommandSegment
                grabFromStackOuter = new SequentialCommandSegment(
                    () -> true,
                    () -> null,
                    allianceSide == Robot.Side.BLUE ? () -> robot.claw.setLeftClawClosed(true, null) : () -> robot.claw.setRightClawClosed(true, null),
                    () -> {},
                    () -> robot.claw.setFlip(false, null),
                    allianceSide == Robot.Side.BLUE ? robot.claw::leftClawInPosition : robot.claw::rightClawInPosition,
                    () -> new Rotation2D(0), () -> new Rotation2D(0)),
                grabFromStackInner = new SequentialCommandSegment(
                    () -> true,
                    () -> null,
                    allianceSide == Robot.Side.BLUE ? () -> robot.claw.setRightClawClosed(true, null) : () -> robot.claw.setLeftClawClosed(true, null),
                    () -> {},
                    () -> robot.claw.setFlip(false, null),
                    allianceSide == Robot.Side.BLUE ? robot.claw::leftClawInPosition : robot.claw::rightClawInPosition,
                    () -> new Rotation2D(0), () -> new Rotation2D(0));

        robot.claw.setFlip(true, null);
        robot.claw.setLeftClawClosed(true, null);
        robot.claw.setRightClawClosed(true, null);


        super.commandSegments.add(sequentialCommandFactory.calibratePositionEstimator());
        super.commandSegments.add(teamElementFinder.findTeamElementAndShutDown(5000));
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
                        () -> {}, () -> {},
                        splitPreload,
                        robot.chassis::isCurrentTranslationalTaskRoughlyComplete,
                        robot.positionEstimator::getRotation2D, () -> new Rotation2D(0)
                )
        );

        super.commandSegments.add(sequentialCommandFactory.waitFor(300));

        super.commandSegments.add(sequentialCommandFactory.justDoIt(() ->
                robot.arm.setPosition(RobotConfig.ArmConfigs.Position.SCORE, null)));

        super.commandSegments.add(sequentialCommandFactory.waitFor(200));

        super.commandSegments.add(sequentialCommandFactory.moveToPoint(
                sequentialCommandFactory.getBezierCurvesFromPathFile("score second").get(0).getPositionWithLERP(1),
                () -> {
                    robot.arm.setScoringHeight(0.25, null);
                    robot.claw.setLeftClawClosed(true, null);
                    robot.claw.setRightClawClosed(true, null);
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

        super.commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> sequentialCommandFactory.getBezierCurvesFromPathFile("move back and grab third from stack").get(0),
                () -> {
                    robot.claw.setFlip(false, null);
                    robot.arm.setPosition(RobotConfig.ArmConfigs.Position.GRAB_STACK, null);
                },
                () -> {},
                () -> robot.claw.setFlip(true, null),
                robot.arm::isArmInPosition,
                () -> new Rotation2D(0), () -> new Rotation2D(0),
                SpeedCurves.originalSpeed, speedConstrainWhenArmRaised
        ));

        super.commandSegments.add(sequentialCommandFactory.followSingleCurveAndStop(
                "move back and grab third from stack", 1,
                new Rotation2D(0)
        ));

        super.commandSegments.add(grabFromStackOuter);
        super.commandSegments.add(sequentialCommandFactory.waitFor(200)); // wait for servo

        super.commandSegments.addAll(sequentialCommandFactory.followPathFacing(
                "grab fourth from stack",
                new Rotation2D(0),
                () -> {}, () -> {}, () -> robot.claw.setFlip(true, null)
        ));
        super.commandSegments.add(sequentialCommandFactory.waitFor(300)); // wait for servo
        super.commandSegments.add(grabFromStackInner);

        super.commandSegments.add(sequentialCommandFactory.followSingleCurve(
                "score third and fourth", 0,
                new Rotation2D(0)
        ));

        super.commandSegments.add(sequentialCommandFactory.followSingleCurve(
                "score third and fourth", 1,
                new Rotation2D(0),
                () -> {
                    robot.arm.setPosition(RobotConfig.ArmConfigs.Position.SCORE, null);
                    robot.arm.setScoringHeight(0.5, null);
                }, () -> {}, () -> {},
                SpeedCurves.slowDown, 0.5
        ));

        super.commandSegments.add(wallAimBot.stickToWall());
        super.commandSegments.add(sequentialCommandFactory.justDoIt(() -> {
            robot.claw.setLeftClawClosed(false, null);
            robot.claw.setRightClawClosed(false, null);
        }));

        if (1==1)
            return;

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
