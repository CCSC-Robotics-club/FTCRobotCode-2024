package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Arm;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;

import java.util.function.DoubleSupplier;

// TODO: re-write the whole program as status machine

public class AprilTagCameraAndDistanceSensorAimBot {
    private final Chassis chassis;
    private final DoubleSupplier desiredDistanceToWallSuplier;
    private final DistanceSensor distanceSensor;
    private final FixedAngleArilTagCamera aprilTagCamera;
    private final ModulesCommanderMarker modulesCommanderMarker;
    private final TelemetrySender telemetrySender;
    private Vector2D previousWallPosition = new Vector2D(new double[]{0, 0});
    private double previousWallDistance = Double.POSITIVE_INFINITY;
    public AprilTagCameraAndDistanceSensorAimBot(Chassis chassis, DistanceSensor distanceSensor, FixedAngleArilTagCamera aprilTagCamera, ModulesCommanderMarker commanderMarker) {
        this(chassis, distanceSensor, aprilTagCamera, null, commanderMarker, null);
    }
    public AprilTagCameraAndDistanceSensorAimBot(Chassis chassis, DistanceSensor distanceSensor, FixedAngleArilTagCamera aprilTagCamera, Arm arm, ModulesCommanderMarker commanderMarker, TelemetrySender telemetrySender) {
        this.chassis = chassis;
        this.distanceSensor = distanceSensor;
        this.aprilTagCamera = aprilTagCamera;
        this.modulesCommanderMarker = commanderMarker;
        this.telemetrySender = telemetrySender;
        this.desiredDistanceToWallSuplier = arm != null ?
                arm::getScoringDistanceToWall : RobotConfig.VisualNavigationConfigs.targetedRelativePositionToWallPreciseTOFApproach::getY;
    }

    public SequentialCommandSegment stickToWall() {
        return stickToWall(getWallPosition(TeamElementFinder.TeamElementPosition.CENTER));
    }

    public SequentialCommandSegment stickToWall(Vector2D desiredPositionToWall) {
        return new SequentialCommandSegment(
                () -> true,
                null,
                this::init,
                () -> this.update(desiredPositionToWall),
                () -> chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY, new Vector2D()), modulesCommanderMarker),
                chassis::isCurrentTranslationalTaskComplete,
                () -> new Rotation2D(0), () -> new Rotation2D(0)
        );
    }

    public SequentialCommandSegment stickToWall(TeamElementFinder teamElementFinder, SequentialCommandSegment.InitiateCondition initiateCondition) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> null,
                this::init,
                () -> this.update(getWallPosition(teamElementFinder.getTeamElementPosition())),
                () -> chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY, new Vector2D()), modulesCommanderMarker),
                chassis::isCurrentTranslationalTaskComplete,
                () -> new Rotation2D(0), () -> new Rotation2D(0)
        );
    }

    public Vector2D getWallPosition(TeamElementFinder.TeamElementPosition teamElementPosition) {
        final double deviationFromCenter;
        switch (teamElementPosition) {
            case UNDETERMINED: case LEFT: {
                deviationFromCenter = -RobotConfig.VisualNavigationConfigs.autoStageScoringHorizontalDeviation;
                break;
            }
            case RIGHT: {
                deviationFromCenter = RobotConfig.VisualNavigationConfigs.autoStageScoringHorizontalDeviation;
                break;
            }
            case CENTER: {
                deviationFromCenter = 0;
                break;
            }
            default: throw new IllegalStateException("unknown team element result: " + teamElementPosition);
        }
        return new Vector2D(new double[] {
                RobotConfig.VisualNavigationConfigs.targetedRelativePositionToWallPreciseTOFApproach.getX() + deviationFromCenter,
                desiredDistanceToWallSuplier.getAsDouble()
        });
    }

    boolean distanceSensorTrustable = true;

    private void init() {
        final long t0 = System.currentTimeMillis();
        while (!chassis.isVisualNavigationAvailable() && System.currentTimeMillis() - t0 < RobotConfig.VisualNavigationConfigs.maxTimeToWaitForVisualNavigationMS * 10) {
            chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY, new Vector2D()), modulesCommanderMarker);
            chassis.periodic();
            aprilTagCamera.periodic();
            chassis.forceUpdateEncoders(modulesCommanderMarker);
            chassis.forceUpdateWheels(modulesCommanderMarker);
            try { Thread.sleep(50); } catch (InterruptedException ignored) {}
        }
        if (!chassis.isVisualNavigationAvailable()) return;
        final double distanceSensorReading = distanceSensor.getDistance(DistanceUnit.CM);
        if (distanceSensorReading > RobotConfig.VisualNavigationConfigs.distanceSensorMaxDistance)
            System.out.println("<-- warning: target might be too far -->");
        resetAimBot();
        updateWallPositionTOF();
    }

    private void update(Vector2D desiredPositionToWall) {
        this.updateWallPositionTOF();
        chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                previousWallPosition.addBy(desiredPositionToWall)
        ), modulesCommanderMarker);
    }

    private void updateWallPositionTOF() {
        final double distanceSensorReading = distanceSensor.getDistance(DistanceUnit.CM),
                newWallPositionX = chassis.isVisualNavigationAvailable() ?
                        chassis.getChassisEncoderPosition().getX() - chassis.getRelativeFieldPositionToWall().getX():
                        previousWallPosition.getX(),
                newWallPositionY = distanceSensorTrustable ?
                        chassis.getChassisEncoderPosition().getY() + distanceSensorReading :
                        previousWallPosition.getY();
        final Vector2D newWallPosition = new Vector2D(
                new double[]{newWallPositionX, newWallPositionY});

        if (telemetrySender!=null) {
            telemetrySender.putSystemMessage("tof trustable", distanceSensorTrustable);
            telemetrySender.putSystemMessage("visual available", chassis.isVisualNavigationAvailable());
            telemetrySender.putSystemMessage("new wall position", newWallPosition);
            telemetrySender.putSystemMessage("previous wall position", previousWallPosition);
        }

        if (Vector2D.displacementToTarget(previousWallPosition, newWallPosition).getMagnitude() > RobotConfig.VisualNavigationConfigs.errorTolerance / 2)
            previousWallPosition = newWallPosition; // only update if outside tolerance

        /* to see if the distance sensor is failing, some times if the robot shifted too much to the side, the distance sensor loses contact with the target and reports a very far distance */
        double change = distanceSensorReading - previousWallDistance;
        if (change > RobotConfig.VisualNavigationConfigs.approachReverseSpeedTolerance)
            distanceSensorTrustable = false;
    }

    private void resetAimBot() {
        distanceSensorTrustable = true;
    }
}
