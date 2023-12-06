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
import org.firstinspires.ftc.teamcode.Utils.AngleUtils;
import org.firstinspires.ftc.teamcode.Utils.AprilTagCameraAndDistanceSensorAimBot;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.BezierCurve;
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

    @Override
    public void scheduleCommands(Chassis chassis, DistanceSensor distanceSensor, FixedAngleArilTagCamera aprilTagCamera, Arm arm, Intake intake, FixedAnglePixelCamera pixelCamera, ModulesCommanderMarker commanderMarker, TelemetrySender telemetrySender) {
        final TeamElementFinder teamElementFinder = new TeamElementFinder(chassis, distanceSensor);
        final AprilTagCameraAndDistanceSensorAimBot aimBot = new AprilTagCameraAndDistanceSensorAimBot(chassis, distanceSensor, aprilTagCamera, commanderMarker, telemetrySender);
        final PixelCameraAimBot pixelCameraAimBot = new PixelCameraAimBot(chassis, pixelCamera, commanderMarker, new HashMap<>());

        /* move to the scanning position */
        BezierCurve path = new BezierCurve(new Vector2D(), constantsTable.scanTeamElementPosition);
        commandSegments.add(new SequentialCommandSegment(
                path,
                () -> {
                    chassis.setCurrentYaw(constantsTable.startingRobotFacing);
                },
                () -> {},
                () -> {},
                () -> true,
                constantsTable.startingRobotFacing,
                constantsTable.startingRobotFacing + Objects.requireNonNull(RobotConfig.TeamElementFinderConfigs.teamElementPositionSearchRotationRanges.get(TeamElementFinder.TeamElementPosition.LEFT))[0]
        ));

        /* scan for team element */
        commandSegments.add(teamElementFinder.getDistanceSensorFindingCommand(TeamElementFinder.TeamElementPosition.LEFT, constantsTable.centerTeamElementRotation));
        commandSegments.add(teamElementFinder.getDistanceSensorFindingCommand(TeamElementFinder.TeamElementPosition.CENTER, constantsTable.centerTeamElementRotation));
        commandSegments.add(teamElementFinder.getDistanceSensorFindingCommand(TeamElementFinder.TeamElementPosition.RIGHT, constantsTable.centerTeamElementRotation));


        telemetrySender.putSystemMessage("element position", teamElementFinder.getFindingResult());
        commandSegments.add(new SequentialCommandSegment(
                path,
                () -> {},
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
                        () -> new BezierCurve(new Vector2D(), constantsTable.getReleasePixelLinePosition(teamElementFinder.getFindingResult())),
                        () -> {},
                        () -> {},
                        () -> {},
                        () -> true,
                        () -> constantsTable.startingRobotFacing,
                        () -> constantsTable.getReleasePixelRotation(teamElementFinder.getFindingResult()) + Math.PI // feeding is in the back end
                )
        );

        /* place the pixel in place, and leave, face front*/
        commandSegments.add(
                new SequentialCommandSegment(
                        () -> teamElementFinder.getFindingResult() != TeamElementFinder.TeamElementPosition.UNDETERMINED,
                        () -> new BezierCurve(new Vector2D(), constantsTable.getReleasePixelLinePosition(teamElementFinder.getFindingResult())),
                        () -> {
                            intake.setMotion(Intake.Motion.REVERSE, commanderMarker);
                        },
                        () -> {},
                        () -> {
                            intake.setMotion(Intake.Motion.STOP, commanderMarker);
                        },
                        () -> true,
                        () -> constantsTable.getReleasePixelRotation(teamElementFinder.getFindingResult()) + Math.PI,
                        () -> 0 // face front
                )
        );
    }


    public static final class AutoStageConstantsTables {
        public static final AutoStageConstantsTable blueAllianceFrontField = new AutoStageConstantsTable( // first we work on this one
                Robot.Side.BLUE,
                false,
                0,
                -Math.PI / 2,
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0})
        );

        public static final AutoStageConstantsTable blueAllianceBackField = new AutoStageConstantsTable(
                Robot.Side.BLUE,
                true,
                0,
                -Math.PI / 2,
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0})
        );


        public static final AutoStageConstantsTable redAllianceFrontField = new AutoStageConstantsTable(
                Robot.Side.RED,
                false,
                Math.PI,
                Math.PI / 2,
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0})
        );

        public static final AutoStageConstantsTable redAllianceBackField = new AutoStageConstantsTable(
                Robot.Side.RED,
                true,
                Math.PI,
                Math.PI / 2,
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0}), new Vector2D(new double[] {0,0})
        );
    }
}