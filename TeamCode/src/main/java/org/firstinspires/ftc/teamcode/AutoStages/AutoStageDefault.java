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

    private long teamElementFinderTimer= -1;
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
                },
                () -> {},
                () -> {},
                () -> true,
                constantsTable.startingRobotFacing,
                constantsTable.centerTeamElementRotation + RobotConfig.TeamElementFinderConfigs.searchRotation
        ));

        /* scan left side */
        commandSegments.add(new SequentialCommandSegment(
                null,
                () -> {
                    teamElementFinderTimer = System.currentTimeMillis();
                },
                () -> {
                    teamElementFinder.search(TeamElementFinder.TeamElementPosition.LEFT);
                },
                () -> {},
                () -> teamElementFinderTimer > RobotConfig.TeamElementFinderConfigs.timeOut || teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
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
                () -> teamElementFinderTimer > RobotConfig.TeamElementFinderConfigs.timeOut || teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
                constantsTable.centerTeamElementRotation + RobotConfig.TeamElementFinderConfigs.searchRotation, constantsTable.centerTeamElementRotation
        ));
        /* scan center */
        commandSegments.add(new SequentialCommandSegment(
                () -> teamElementFinder.getFindingResult() == TeamElementFinder.TeamElementPosition.UNDETERMINED,
                null,
                () -> {
                    teamElementFinderTimer = System.currentTimeMillis();
                },
                () -> {
                    teamElementFinder.search(TeamElementFinder.TeamElementPosition.CENTER);
                },
                () -> {},
                () -> teamElementFinderTimer > RobotConfig.TeamElementFinderConfigs.timeOut || teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
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
                () -> teamElementFinderTimer > RobotConfig.TeamElementFinderConfigs.timeOut || teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
                constantsTable.centerTeamElementRotation, constantsTable.centerTeamElementRotation - RobotConfig.TeamElementFinderConfigs.searchRotation
        ));
        /* scan right side */
        commandSegments.add(new SequentialCommandSegment(
                () -> teamElementFinder.getFindingResult() == TeamElementFinder.TeamElementPosition.UNDETERMINED,
                null,
                () -> {
                    teamElementFinderTimer = System.currentTimeMillis();
                },
                () -> {
                    teamElementFinder.search(TeamElementFinder.TeamElementPosition.RIGHT);
                },
                () -> {},
                () -> teamElementFinderTimer > RobotConfig.TeamElementFinderConfigs.timeOut || teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
                constantsTable.centerTeamElementRotation - RobotConfig.TeamElementFinderConfigs.searchRotation, constantsTable.centerTeamElementRotation - RobotConfig.TeamElementFinderConfigs.searchRotation
        ));


        commandSegments.add(new SequentialCommandSegment(
                null,
                () -> {
                    telemetrySender.putSystemMessage("element position", teamElementFinder.getFindingResult());
                },
                () -> {},
                () -> {},
                () -> false,
                constantsTable.startingRobotFacing + Objects.requireNonNull(RobotConfig.TeamElementFinderConfigs.teamElementPositionSearchRotationRanges.get(TeamElementFinder.TeamElementPosition.LEFT))[0],
                constantsTable.startingRobotFacing + Objects.requireNonNull(RobotConfig.TeamElementFinderConfigs.teamElementPositionSearchRotationRanges.get(TeamElementFinder.TeamElementPosition.LEFT))[0]
        )); // wait forever TODO test the part before this



        /* if there is an result, drive there */
        commandSegments.add(
                new SequentialCommandSegment(
                        () -> teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
                        () -> new BezierCurve(constantsTable.scanTeamLeftRightElementPosition, constantsTable.getReleasePixelLinePosition(teamElementFinder.getFindingResult())),
                        () -> {},
                        () -> {},
                        () -> {},
                        () -> true,
                        () -> constantsTable.startingRobotFacing,
                        () -> constantsTable.getReleasePixelRotation(teamElementFinder.getFindingResult()) + Math.PI // feeding is in the back end
                )
        );

//        /* place the pixel in place, and leave, face front*/
//        commandSegments.add(
//                new SequentialCommandSegment(
//                        () -> teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
//                        () -> new BezierCurve(constantsTable.getReleasePixelLinePosition(teamElementFinder.getFindingResult()), constantsTable.scanTeamElementPosition),
//                        () -> {
//                            intake.setMotion(Intake.Motion.REVERSE, commanderMarker);
//                        },
//                        () -> {},
//                        () -> {
//                            intake.setMotion(Intake.Motion.STOP, commanderMarker);
//                        },
//                        () -> true,
//                        () -> constantsTable.getReleasePixelRotation(teamElementFinder.getFindingResult()) + Math.PI,
//                        () -> 0 // face front
//                )
//        );
//
//        /* if we are at the back filed, drive to front field */
//        path = new BezierCurve(
//                constantsTable.scanTeamElementPosition,
//                new Vector2D(),
//                new Vector2D(new double[] {constantsTable.lowestHorizontalWalkWayAndOutMostVerticalWalkWayCross.getX(), constantsTable.centerLineYPosition}),
//                new Vector2D(new double[] {0, 0})
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
                new Vector2D(new double[] {0, 0}), new Vector2D(new double[] {0, 0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
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