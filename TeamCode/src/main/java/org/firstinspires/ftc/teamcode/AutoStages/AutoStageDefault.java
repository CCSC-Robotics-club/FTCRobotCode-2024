package org.firstinspires.ftc.teamcode.AutoStages;

import com.qualcomm.robotcore.hardware.DistanceSensor;

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

    private long teamElementFinderTimer= -1, spewPixelTimer = -1;
    @Override
    public void scheduleCommands(Chassis chassis, DistanceSensor distanceSensor, FixedAngleArilTagCamera aprilTagCamera, Arm arm, Intake intake, FixedAnglePixelCamera pixelCamera, ModulesCommanderMarker commanderMarker, TelemetrySender telemetrySender) {
        final TeamElementFinder teamElementFinder = new TeamElementFinder(chassis, distanceSensor, (HuskyAprilTagCamera) aprilTagCamera.getRawAprilTagCamera());
        final AprilTagCameraAndDistanceSensorAimBot aimBot = new AprilTagCameraAndDistanceSensorAimBot(chassis, distanceSensor, aprilTagCamera, commanderMarker, telemetrySender);
        final PixelCameraAimBot pixelCameraAimBot = new PixelCameraAimBot(chassis, pixelCamera, commanderMarker, new HashMap<>());

        /* move to the scanning position */
        BezierCurve path = new BezierCurve(new Vector2D(), constantsTable.scanTeamLeftRightElementPosition);
        commandSegments.add(new SequentialCommandSegment(
                path,
                () -> {
                    chassis.setCurrentYaw(constantsTable.startingRobotFacing);
                    arm.gainOwnerShip(commanderMarker);
                    intake.gainOwnerShip(commanderMarker);
                },
                () -> {},
                () -> {

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
                        () -> new BezierCurve(constantsTable.scanTeamLeftRightElementPosition, constantsTable.getReleasePixelLinePosition(teamElementFinder.getFindingResult())),
                        () -> {
                            telemetrySender.putSystemMessage("starting rotation of segment 6", Math.toDegrees(chassis.getYaw()));
                            telemetrySender.putSystemMessage("ending rotation of segment 6", Math.toDegrees(constantsTable.getReleasePixelRotation(teamElementFinder.getFindingResult()) + Math.PI));
                        },
                        () -> {},
                        () -> {
                        },
                        () -> chassis.isCurrentRotationalTaskComplete() && chassis.isCurrentTranslationalTaskComplete(), // make sure it is precise
                        chassis::getYaw,
                        () -> constantsTable.getReleasePixelRotation(teamElementFinder.getFindingResult()) + Math.PI // feeding is in the back end
                )
        );

        /* place the pixel in place, and leave, face front*/
        commandSegments.add(
                new SequentialCommandSegment(
                        () -> teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
                        () -> new BezierCurve(
                                constantsTable.getReleasePixelLinePosition(teamElementFinder.getFindingResult()),
                                constantsTable.getReleasePixelLinePosition(teamElementFinder.getFindingResult()).addBy(
                                        new Vector2D(new double[] {0, -RobotConfig.IntakeConfigs.spewPixelDriveBackDistance})
                                                .multiplyBy(new Rotation2D(constantsTable.getReleasePixelRotation(teamElementFinder.getFindingResult()) + Math.PI)) // drive back a little
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
                        () -> constantsTable.getReleasePixelRotation(teamElementFinder.getFindingResult()) + Math.PI,
                        () -> constantsTable.getReleasePixelRotation(teamElementFinder.getFindingResult()) + Math.PI // face front
                )
        );

        /* if we are at the back filed, drive to front field */
//        path = new BezierCurve(
//                constantsTable.scanTeamLeftRightElementPosition,
//                new Vector2D(),
//                new Vector2D(),
//                new Vector2D(new double[] {constantsTable.lowestHorizontalWalkWayAndOutMostVerticalWalkWayCross.getX(), constantsTable.centerLineYPosition})
//        );
//        commandSegments.add(
//                new SequentialCommandSegment(
//                        () -> constantsTable.backField,
//                        path,
//                        () -> {},
//                        () -> {},
//                        () -> {},
//                        () -> true,
//                        0, 0
//                )
//        );

        /* TODO next, go to the wall */
    }

    public static final class AutoStageConstantsTables {
        public static final AutoStageConstantsTable blueAllianceFrontField = new AutoStageConstantsTable( // first we work on this one
                Robot.Side.BLUE,
                false,
                0,
                -Math.PI / 2,
                0,
                new Vector2D(new double[] {48, 0}), new Vector2D(new double[] {65, 0}), new Vector2D(new double[] {48,40}), new Vector2D(new double[] {95,30}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0})
        );

        public static final AutoStageConstantsTable blueAllianceBackField = new AutoStageConstantsTable(
                Robot.Side.BLUE,
                true,
                0,
                -Math.PI / 2,
                0,
                new Vector2D(new double[] {0, 0}), new Vector2D(new double[] {0, 0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0})
        );


        public static final AutoStageConstantsTable redAllianceFrontField = new AutoStageConstantsTable(
                Robot.Side.RED,
                false,
                Math.PI,
                Math.PI / 2,
                0,
                new Vector2D(new double[] {0, 0}), new Vector2D(new double[] {0, 0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0})
        );

        public static final AutoStageConstantsTable redAllianceBackField = new AutoStageConstantsTable(
                Robot.Side.RED,
                true,
                Math.PI,
                Math.PI / 2,
                0,
                new Vector2D(new double[] {0, 0}), new Vector2D(new double[] {0, 0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0})
        );
    }
}