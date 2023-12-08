package org.firstinspires.ftc.teamcode.AutoStages;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Modules.Arm;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Modules.FixedAnglePixelCamera;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AprilTagCameraAndDistanceSensorAimBot;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.HuskyAprilTagCamera;
import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.PixelCameraAimBot;
import org.firstinspires.ftc.teamcode.Utils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.TeamElementFinder;
import org.firstinspires.ftc.teamcode.Utils.Vector2D;

import java.nio.file.ClosedWatchServiceException;
import java.util.HashMap;
import java.util.Objects;

public class AutoStageDefault extends AutoStageProgram {
    private final boolean frontField;
    private final AutoStageConstantsTable constantsTable;
    public AutoStageDefault(AutoStageConstantsTable constantsTable, boolean frontField) {
        super(constantsTable.allianceSide);
        this.frontField = frontField;
        this.constantsTable = constantsTable;
    }

    private long teamElementFinderTimer= -1, spewPixelTimer = -1, servoTimer = -1;
    @Override
    public void scheduleCommands(Chassis chassis, DistanceSensor distanceSensor, FixedAngleArilTagCamera aprilTagCamera, Arm arm, Intake intake, FixedAnglePixelCamera pixelCamera, ModulesCommanderMarker commanderMarker, TelemetrySender telemetrySender) {
        final TeamElementFinder teamElementFinder = new TeamElementFinder(chassis, distanceSensor, (HuskyAprilTagCamera) aprilTagCamera.getRawAprilTagCamera());
        final AprilTagCameraAndDistanceSensorAimBot wallAimBot = new AprilTagCameraAndDistanceSensorAimBot(chassis, distanceSensor, aprilTagCamera, commanderMarker, telemetrySender);
        final PixelCameraAimBot pixelCameraAimBot = new PixelCameraAimBot(chassis, pixelCamera, commanderMarker, new HashMap<>());

        /* move to the scanning position */
        BezierCurve path = new BezierCurve(new Vector2D(), constantsTable.scanTeamLeftRightElementPosition);
        commandSegments.add(new SequentialCommandSegment(
                path,
                () -> {
                    chassis.gainOwnerShip(commanderMarker);
                    chassis.setCurrentYaw(constantsTable.startingRobotFacing);
                    arm.gainOwnerShip(commanderMarker);
                    intake.gainOwnerShip(commanderMarker);
                    arm.setArmCommand(new Arm.ArmCommand(Arm.ArmCommand.ArmCommandType.SET_POSITION, RobotConfig.ArmConfigs.lowPos), commanderMarker);
                },
                () -> {},
                () -> {
                    arm.setArmCommand(new Arm.ArmCommand(Arm.ArmCommand.ArmCommandType.SET_MOTOR_POWER, 0), commanderMarker);
                },
                () -> true,
                constantsTable.startingRobotFacing,
                constantsTable.centerTeamElementRotation + RobotConfig.TeamElementFinderConfigs.searchRotation
        ));

        /* scan left side */
        commandSegments.add(new SequentialCommandSegment(
                null,
                () -> {
                    teamElementFinderTimer = System.currentTimeMillis();
                    teamElementFinder.startSearch();
                },
                () -> {
                    teamElementFinder.proceedSearch(TeamElementFinder.TeamElementPosition.LEFT);
                },
                teamElementFinder::stopSearch,
                () -> System.currentTimeMillis() - teamElementFinderTimer > RobotConfig.TeamElementFinderConfigs.timeOut || teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
                constantsTable.centerTeamElementRotation + RobotConfig.TeamElementFinderConfigs.searchRotation, constantsTable.centerTeamElementRotation + RobotConfig.TeamElementFinderConfigs.searchRotation
        ));
        /* face center */
        path = new BezierCurve(constantsTable.scanTeamLeftRightElementPosition, constantsTable.scanTeamCenterElementPosition);
        commandSegments.add(new SequentialCommandSegment(
                () -> teamElementFinder.getFindingResult() == TeamElementFinder.TeamElementPosition.UNDETERMINED,
                path,
                () -> {},
                () -> {},
                () -> {},
                () -> true,
                constantsTable.centerTeamElementRotation + RobotConfig.TeamElementFinderConfigs.searchRotation, constantsTable.centerTeamElementRotation
        ));
        /* scan center */
        commandSegments.add(new SequentialCommandSegment(
                () -> teamElementFinder.getFindingResult() == TeamElementFinder.TeamElementPosition.UNDETERMINED,
                null,
                () -> {
                    teamElementFinderTimer = System.currentTimeMillis();
                    teamElementFinder.startSearch();
                },
                () -> {
                    teamElementFinder.proceedSearch(TeamElementFinder.TeamElementPosition.CENTER);
                },
                teamElementFinder::stopSearch,
                () -> System.currentTimeMillis() - teamElementFinderTimer > RobotConfig.TeamElementFinderConfigs.timeOut || teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
                constantsTable.centerTeamElementRotation, constantsTable.centerTeamElementRotation
        ));
        /* face right side */
        path = new BezierCurve(constantsTable.scanTeamCenterElementPosition, constantsTable.scanTeamLeftRightElementPosition);
        commandSegments.add(new SequentialCommandSegment(
                () -> teamElementFinder.getFindingResult() == TeamElementFinder.TeamElementPosition.UNDETERMINED,
                path,
                () -> {},
                () -> {},
                () -> {},
                () -> true,
                constantsTable.centerTeamElementRotation, constantsTable.centerTeamElementRotation - RobotConfig.TeamElementFinderConfigs.searchRotation
        ));
        /* scan right side */
        commandSegments.add(new SequentialCommandSegment(
                () -> teamElementFinder.getFindingResult() == TeamElementFinder.TeamElementPosition.UNDETERMINED,
                null,
                () -> {
                    teamElementFinderTimer = System.currentTimeMillis();
                    teamElementFinder.startSearch();
                },
                () -> {
                    teamElementFinder.proceedSearch(TeamElementFinder.TeamElementPosition.RIGHT);
                },
                teamElementFinder::stopSearch,
                () -> System.currentTimeMillis() - teamElementFinderTimer > RobotConfig.TeamElementFinderConfigs.timeOut || teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
                constantsTable.centerTeamElementRotation - RobotConfig.TeamElementFinderConfigs.searchRotation, constantsTable.centerTeamElementRotation - RobotConfig.TeamElementFinderConfigs.searchRotation
        ));

        /* if there is an result, drive there */
        commandSegments.add(
                new SequentialCommandSegment(
                        () -> teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
                        () -> new BezierCurve(
                                constantsTable.scanTeamLeftRightElementPosition,
                                constantsTable.scanTeamCenterElementPosition,
                                constantsTable.scanTeamCenterElementPosition,
                                constantsTable.getReleasePixelLinePosition(teamElementFinder.getFindingResult())),
                        () -> {
                            // arm.setArmCommand(new Arm.ArmCommand(Arm.ArmCommand.ArmCommandType.SET_POSITION, 0), commanderMarker);
                        },
                        () -> {},
                        () -> {
                        },
                        () -> chassis.isCurrentRotationalTaskComplete() && chassis.isCurrentTranslationalTaskComplete(), // make sure it is precise
                        // ()->true,
                        chassis::getYaw,
                        constantsTable::getReleasePixelRotation // feeding is in the back end
                )
        );

        /* place the pixel in place, and leave, face front*/
        commandSegments.add(
                new SequentialCommandSegment(
                        () -> teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
                        () -> new BezierCurve(
                                constantsTable.getReleasePixelLinePosition(teamElementFinder.getFindingResult()),
                                constantsTable.getReleasePixelLinePosition(teamElementFinder.getFindingResult()).addBy(
                                        new Vector2D(new double[] {0, RobotConfig.IntakeConfigs.spewPixelDriveBackDistance})
                                                .multiplyBy(new Rotation2D(constantsTable.getReleasePixelRotation())) // drive back a little
                                )),
                        () -> {
                            intake.setMotion(Intake.Motion.REVERSE, commanderMarker);
                            spewPixelTimer = System.currentTimeMillis();
                        },
                        () -> {},
                        () -> {
                            intake.setMotion(Intake.Motion.STOP, commanderMarker);
                        },
                        () -> System.currentTimeMillis() - spewPixelTimer > RobotConfig.IntakeConfigs.spewPixelTimeMillis,
                        constantsTable::getReleasePixelRotation,
                        constantsTable::getReleasePixelRotation
                )
        );

        // TODO below this line are those waiting to be tested:

        /* if we are at the back filed, drive to front field */
        commandSegments.add(
                new SequentialCommandSegment(
                        () -> constantsTable.backField,
                        () -> new BezierCurve(
                                chassis.getChassisEncoderPosition(),
                                new Vector2D(new double[] {
                                        constantsTable.lowestHorizontalWalkWayAndOutMostVerticalWalkWayCross.getX(),
                                        0
                                }),
                                new Vector2D(new double[]{
                                                constantsTable.lowestHorizontalWalkWayAndOutMostVerticalWalkWayCross.getX(),
                                                constantsTable.centerLineYPosition
                                })
                        ),
                        () -> {
                            arm.holdPixel(commanderMarker);
                        },
                        () -> {},
                        () -> {},
                        () -> true,
                        constantsTable::getReleasePixelRotation,
                        () -> 0
                )
        );

        /* if we are at back-field, drive there with a smooth path */
        SequentialCommandSegment.BezierCurveFeeder bezierCurveFeeder;
        if (constantsTable.backField)
            bezierCurveFeeder =
                    () -> new BezierCurve(
                    new Vector2D(new double[]{
                            constantsTable.lowestHorizontalWalkWayAndOutMostVerticalWalkWayCross.getX(),
                            constantsTable.centerLineYPosition
                    }),
                    new Vector2D(new double[] {
                            constantsTable.lowestHorizontalWalkWayAndOutMostVerticalWalkWayCross.getX(),
                            constantsTable.aimWallSweetSpot.getY()
                    }),
                    constantsTable.aimWallSweetSpot
            );
        /* otherwise, just drive there in a straight line */
        else
            bezierCurveFeeder =
                    () -> new BezierCurve(
                            chassis.getChassisEncoderPosition(),
                            constantsTable.aimWallSweetSpot
                    );
        commandSegments.add(
                new SequentialCommandSegment(
                        () -> constantsTable.backField,
                        bezierCurveFeeder,
                        () -> {
                            arm.setArmCommand(new Arm.ArmCommand(Arm.ArmCommand.ArmCommandType.SET_POSITION, RobotConfig.ArmConfigs.lowPos), commanderMarker);
                        },
                        () -> {},
                        () -> {},
                        () -> true,
                        constantsTable::getReleasePixelRotation,
                        () -> 0
                )
        );

        /* aim and place the first pixel */
        commandSegments.add(wallAimBot.createCommandSegment(teamElementFinder, () -> true));
        commandSegments.add(
                new SequentialCommandSegment(
                        null,
                        () -> {
                            arm.placePixel(commanderMarker);
                            servoTimer = System.currentTimeMillis();
                        },
                        () -> {},
                        () -> {
                            arm.setArmCommand(new Arm.ArmCommand(Arm.ArmCommand.ArmCommandType.SET_POSITION, RobotConfig.ArmConfigs.feedPos), commanderMarker);
                        },
                        () -> System.currentTimeMillis() - servoTimer > RobotConfig.ArmConfigs.extendTime * 2,
                        0, 0
                )
        );

        commandSegments.add(
                new SequentialCommandSegment(
                        () -> true,
                        () -> null,
                        () -> {},
                        () -> {},
                        () -> {},
                        ()->false,
                        chassis::getYaw,
                        chassis::getYaw
                )
        );

        // TODO here, push the new pixel from the intake to the claw and place it if no result found
//        commandSegments.add(
//                new SequentialCommandSegment(
//                        () -> teamElementFinder.getFindingResult() == TeamElementFinder.TeamElementPosition.UNDETERMINED,
//                        null,
//                        () -> {
//                            arm.holdPixel(commanderMarker);
//                            servoTimer = System.currentTimeMillis();
//                        },
//                        () -> {},
//                        () -> {},
//                        () -> System.currentTimeMillis() - servoTimer > RobotConfig.ArmConfigs.extendTime,
//                        0, 0
//                )
//        );

        /* take the inner path to go back */
        commandSegments.add(
                new SequentialCommandSegment(
                        new BezierCurve(
                                constantsTable.aimWallSweetSpot,
                                new Vector2D(new double[] {constantsTable.lowestHorizontalWalkWayAndInnerVerticalWalkWayCross.getX(), constantsTable.aimWallSweetSpot.getY()}),
                                new Vector2D(new double[] {constantsTable.lowestHorizontalWalkWayAndInnerVerticalWalkWayCross.getX(), constantsTable.centerLineYPosition})
                        ),
                        () -> {
                            arm.setArmCommand(new Arm.ArmCommand(Arm.ArmCommand.ArmCommandType.SET_POSITION, 0), commanderMarker);
                        },
                        () -> {},
                        () -> {},
                        arm::isArmDesiredPositionReached,
                        0, 0
                )
        );

        // TODO take the pixel stacks in the back
    }

    public static final class AutoStageConstantsTables {
        public static final AutoStageConstantsTable blueAllianceFrontField = new AutoStageConstantsTable( // first we work on this one
                Robot.Side.BLUE,
                false,
                0,
                -Math.PI / 2,
                Math.toRadians(90),
                0,
                new Vector2D(new double[] {48, 0}), new Vector2D(new double[] {65, 0}),
                new Vector2D(new double[] {85,45}), new Vector2D(new double[] {100, 27}),new Vector2D(new double[] {80,-8}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0})
        );

        public static final AutoStageConstantsTable blueAllianceBackField = new AutoStageConstantsTable(
                Robot.Side.BLUE,
                true,
                0,
                -Math.PI / 2,
                -Math.toRadians(50),
                0,
                new Vector2D(new double[] {48, 0}), new Vector2D(new double[] {65, 0}),
                new Vector2D(new double[] {48,-40}), new Vector2D(new double[] {95,-30}), new Vector2D(new double[] {53,14}), // TODO left right should reverse
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0})
        );


        public static final AutoStageConstantsTable redAllianceFrontField = new AutoStageConstantsTable(
                Robot.Side.RED,
                false,
                Math.PI,
                Math.PI / 2,
                -Math.toRadians(90),
                0,
                new Vector2D(new double[] {-48, 0}), new Vector2D(new double[] {-65, 0}),
                new Vector2D(new double[] {-80,-8}), new Vector2D(new double[] {-100, 27}), new Vector2D(new double[] {-85,45}), // newest
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0})
        );

        public static final AutoStageConstantsTable redAllianceBackField = new AutoStageConstantsTable(
                Robot.Side.RED,
                true,
                Math.PI,
                Math.PI / 2,
                Math.toRadians(50),
                0,
                new Vector2D(new double[] {-48, 0}), new Vector2D(new double[] {-65, 0}),
                new Vector2D(new double[] {-48,-40}), new Vector2D(new double[] {-95,-30}), new Vector2D(new double[] {-53,14}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0})
        );
    }
}