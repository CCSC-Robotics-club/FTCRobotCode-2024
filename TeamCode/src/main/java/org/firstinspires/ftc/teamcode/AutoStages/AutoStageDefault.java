package org.firstinspires.ftc.teamcode.AutoStages;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AngleUtils;
import org.firstinspires.ftc.teamcode.Utils.AprilTagCameraAndDistanceSensorAimBot;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.TeamElementFinder;
import org.firstinspires.ftc.teamcode.Utils.Vector2D;

public class AutoStageDefault extends AutoStageProgram {
    private final boolean frontField, useInnerWalkWay;
    private final AutoStageConstantsTable constantsTable;
    public AutoStageDefault(AutoStageConstantsTable constantsTable, boolean frontField, boolean useInnerWalkWay) {
        super(constantsTable.allianceSide);
        this.frontField = frontField;
        this.useInnerWalkWay =  useInnerWalkWay;
        this.constantsTable = constantsTable;
    }

    long teamElementFindingTimer;
    Vector2D unloadPixelPosition;
    @Override
    public void scheduleCommands(Chassis chassis, DistanceSensor distanceSensor, FixedAngleArilTagCamera aprilTagCamera, ModulesCommanderMarker commanderMarker, TelemetrySender telemetrySender) {
        final TeamElementFinder teamElementFinder = new TeamElementFinder(chassis, distanceSensor, aprilTagCamera.getRawAprilTagCamera());
        final AprilTagCameraAndDistanceSensorAimBot aimBot = new AprilTagCameraAndDistanceSensorAimBot(chassis, distanceSensor, aprilTagCamera, commanderMarker, telemetrySender);

        /* move to the scanning position */
        BezierCurve path = new BezierCurve(new Vector2D(), constantsTable.scanTeamElementPosition);
        commandSegments.add(new SequentialCommandSegment(
                path,
                () -> {},
                () -> {},
                () -> {},
                () -> true,
                constantsTable.startingRobotFacing, constantsTable.startingRobotFacing
        ));

        /* scan for team element */
        commandSegments.add(
                new SequentialCommandSegment(
                        null,
                        () -> {
                            teamElementFindingTimer = System.currentTimeMillis();
                        },
                        teamElementFinder::findElementWithAprilTagCamera,
                        () -> {

                        },
                        () -> System.currentTimeMillis() - teamElementFindingTimer > 2000
                                || teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
                        constantsTable.startingRobotFacing, constantsTable.startingRobotFacing
                )
        );

        if (true) return;

        // TODO: here, if there isn't a result and tof search is needed

        /* if there is an result, drive there and place the pixel */
        commandSegments.add(
                new SequentialCommandSegment(
                        () -> teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
                        null,
                        () -> {
                            chassis.setTranslationalTask(
                                    new Chassis.ChassisTranslationalTask(
                                            Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                                            constantsTable.getFeedingPosition(teamElementFinder.getFindingResult())
                                            ), commanderMarker);
                        },
                        () -> {},
                        () -> {
                            chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY, new Vector2D()), commanderMarker);
                        },
                        () -> chassis.isCurrentTranslationalTaskComplete() && chassis.isCurrentRotationalTaskComplete(),
                        constantsTable.startingRobotFacing, AngleUtils.simplifyAngle(constantsTable.startingRobotFacing + Math.PI) // turn around and feed
                )
        );
        commandSegments.add(
                new SequentialCommandSegment(
                        () -> teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
                        null,
                        () -> {

                        },
                        () -> {},
                        () -> {

                        },
                        () -> chassis.isCurrentTranslationalTaskComplete() && chassis.isCurrentRotationalTaskComplete(),
                        constantsTable.startingRobotFacing, AngleUtils.simplifyAngle(constantsTable.startingRobotFacing + Math.PI) // turn around and feed
                )
        );
    }


    public static final class AutoStageConstantsTables {
        AutoStageConstantsTable blueAllianceFrontField = new AutoStageConstantsTable(
                Robot.Side.BLUE,
                -Math.PI / 2,
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0})
        );

        AutoStageConstantsTable blueAllianceBackField = new AutoStageConstantsTable(
                Robot.Side.BLUE,
                -Math.PI / 2,
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0})
        );


        AutoStageConstantsTable redAllianceFrontField = new AutoStageConstantsTable(
                Robot.Side.RED,
                Math.PI / 2,
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0})
        );

        AutoStageConstantsTable redAllianceBackField = new AutoStageConstantsTable(
                Robot.Side.RED,
                Math.PI / 2,
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0})
        );
    }
}