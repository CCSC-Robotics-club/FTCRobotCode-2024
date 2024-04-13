package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AprilTagCameraAndDistanceSensorAimBot;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.SpeedCurves;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.TeamElementFinder;

public class FrontFieldAutoFourPiecesSloppedGrab extends AutoStageProgram  {
    public FrontFieldAutoFourPiecesSloppedGrab(Robot.Side side) {
        super(side);
    }

    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        final TeamElementFinder teamElementFinder = new TeamElementFinder(robot.hardwareMap.get(WebcamName.class, "Webcam 1"));
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, "split first(left)", new Rotation2D(Math.toRadians(90)), super.allianceSide, robot.hardwareMap);
        final AprilTagCameraAndDistanceSensorAimBot wallAimBot = new AprilTagCameraAndDistanceSensorAimBot(robot.chassis, robot.distanceSensor, robot.aprilTagCamera, robot.arm, null, robot.telemetrySender);
        final double speedConstrainWhenArmRaised = 0.6;
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
                grabFromStack = new SequentialCommandSegment(
                        () -> true,
                        () -> null,
                        () -> {
                            robot.claw.setRightClawClosed(true, null);
                            robot.claw.setLeftClawClosed(true, null);
                        }, () -> {},
                        () -> robot.claw.setFlip(false, null),
                        allianceSide == Robot.Side.BLUE ? robot.claw::leftClawInPosition : robot.claw::rightClawInPosition,
                        () -> null, () -> null);

        robot.claw.setFlip(false, null);
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
                        () -> {},
                        () -> {
                            if (robot.chassis.isCurrentRotationalTaskRoughlyComplete())
                                robot.claw.setFlip(true, null);
                        },
                        () -> robot.claw.setFlip(true, null),
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
                robot.arm::isArmInPosition
        ));

        super.commandSegments.add(sequentialCommandFactory.waitFor(300));

        super.commandSegments.add(sequentialCommandFactory.justDoIt(scorePreload));

        super.commandSegments.add(sequentialCommandFactory.waitFor(500));

        super.commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> sequentialCommandFactory.getBezierCurvesFromPathFile("move back and grab third from stack slopped").get(0),
                () -> {
                    robot.claw.setFlip(false, null);
                    robot.arm.setPosition(RobotConfig.ArmConfigs.Position.GRAB_STACK, null);
                },
                () -> {}, () -> {},
                robot.arm::isArmInPosition,
                () -> new Rotation2D(0), () -> new Rotation2D(0),
                SpeedCurves.originalSpeed, 0.45
        ));

        super.commandSegments.add(sequentialCommandFactory.followSingleCurve(
                "move back and grab third from stack slopped", 1,
                new Rotation2D(Math.toRadians(-30))
        ));
        super.commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> {},
                () -> robot.chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                        Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                        sequentialCommandFactory.getBezierCurvesFromPathFile("move back and grab third from stack slopped").get(1).getPositionWithLERP(1)
                ), null),
                () -> {
                    robot.claw.setFlip(true, null);
                    robot.arm.setPosition(RobotConfig.ArmConfigs.Position.GRAB_STACK_LOW, null);
                },
                () -> robot.chassis.isCurrentTranslationalTaskComplete() && robot.chassis.isCurrentRotationalTaskComplete(),
                () -> new Rotation2D(Math.toRadians(-30)), () -> new Rotation2D(Math.toRadians(-30))
        ));

        super.commandSegments.add(sequentialCommandFactory.waitFor(500)); // wait for servo
        super.commandSegments.add(grabFromStack);
        super.commandSegments.add(sequentialCommandFactory.waitFor(500)); // wait for servo
        super.commandSegments.add(sequentialCommandFactory.justDoIt(() -> {
            robot.claw.setFlip(false, null);
            if (allianceSide == Robot.Side.BLUE) robot.claw.setLeftClawClosed(false, null);
            else robot.claw.setRightClawClosed(false, null);
        }));

        super.commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> sequentialCommandFactory.getBezierCurvesFromPathFile("grab fourth from stack slopped").get(0),
                () -> {}, () -> {}, () -> {},
                robot.chassis::isCurrentTranslationalTaskComplete,
                () -> new Rotation2D(-30), () -> new Rotation2D(Math.toRadians(30)),
                SpeedCurves.easeOut, 0.8
        ));
        super.commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> {},
                () -> robot.chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                        Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                        sequentialCommandFactory.getBezierCurvesFromPathFile("grab fourth from stack slopped").get(0).getPositionWithLERP(1)
                ), null),
                () -> robot.claw.setFlip(true, null),
                () -> robot.chassis.isCurrentTranslationalTaskComplete() && robot.chassis.isCurrentRotationalTaskComplete(),
                () -> new Rotation2D(Math.toRadians(30)), () -> new Rotation2D(Math.toRadians(30))
        ));

        super.commandSegments.add(sequentialCommandFactory.waitFor(500)); // wait for servo
        super.commandSegments.add(grabFromStack);
        super.commandSegments.add(sequentialCommandFactory.waitFor(50000)); // wait for servo

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
                SpeedCurves.easeOut, speedConstrainWhenArmRaised
        ));

        super.commandSegments.add(wallAimBot.stickToWall(robot.arm::isArmInPosition));
        super.commandSegments.add(sequentialCommandFactory.waitFor(800));
        super.commandSegments.add(sequentialCommandFactory.justDoIt(() -> {
            robot.claw.setLeftClawClosed(false, null);
            robot.claw.setRightClawClosed(false, null);
        }));

        // end early
        super.commandSegments.addAll(sequentialCommandFactory.followPathFacing(
                "park",
                new Rotation2D(0),
                () -> robot.arm.setPosition(RobotConfig.ArmConfigs.Position.GRAB_STACK, null),
                ()->{}, ()->{}
        ));if (1==1) return;

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("move to stack grab fifth", new Rotation2D(0))
        );

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("grab sixth from stack", new Rotation2D(0))
        );

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("score fifth and sixth", new Rotation2D(0))
        );

        super.commandSegments.addAll(sequentialCommandFactory.followPathFacing(
                "park",
                new Rotation2D(0),
                () -> robot.arm.setPosition(RobotConfig.ArmConfigs.Position.GRAB_STACK, null),
                ()->{}, ()->{}
        ));
    }
}
