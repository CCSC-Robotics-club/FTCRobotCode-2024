package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Modules.Arm;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.TeamElementFinder;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedSensor;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;

import static org.firstinspires.ftc.teamcode.RobotConfig.VisualNavigationConfigs.*;

import java.util.function.DoubleSupplier;

// TODO: re-write the whole program as status machine

public class AprilTagCameraAndDistanceSensorAimBot {
    private final Chassis chassis;
    private final DoubleSupplier desiredDistanceToWallSupplier;
    private final ThreadedSensor distanceSensor;
    private final FixedAngleArilTagCamera aprilTagCamera;
    private final ModulesCommanderMarker modulesCommanderMarker;
    private final TelemetrySender telemetrySender;
    private final Robot.Side side;
    private Vector2D previousWallPosition = new Vector2D(new double[]{0, 0});
    private double previousWallDistance = Double.POSITIVE_INFINITY;
    public AprilTagCameraAndDistanceSensorAimBot(Chassis chassis, ThreadedSensor distanceSensor, FixedAngleArilTagCamera aprilTagCamera, ModulesCommanderMarker commanderMarker, Robot.Side side) {
        this(chassis, distanceSensor, aprilTagCamera, null, commanderMarker, null, side);
    }
    public AprilTagCameraAndDistanceSensorAimBot(Chassis chassis, ThreadedSensor distanceSensor, FixedAngleArilTagCamera aprilTagCamera, Arm arm, ModulesCommanderMarker commanderMarker, TelemetrySender telemetrySender, Robot.Side side) {
        this.chassis = chassis;
        this.distanceSensor = distanceSensor;
        this.aprilTagCamera = aprilTagCamera;
        this.modulesCommanderMarker = commanderMarker;
        this.telemetrySender = telemetrySender;
        this.desiredDistanceToWallSupplier = () -> RobotConfig.ArmConfigs.distancesToWallAccordingToScoringHeight.getYPrediction(arm.getArmDesiredScoringHeight());
        this.side = side;
    }

    public SequentialCommandSegment stickToWall(SequentialCommandSegment.IsCompleteChecker additionalCompleteChecker) {
        return stickToWall(
                () -> update(getDesiredAimingPositionToWall(TeamElementFinder.TeamElementPosition.CENTER)),
                additionalCompleteChecker
        );
    }

    public SequentialCommandSegment stickToWall(Vector2D desiredPositionToWall, SequentialCommandSegment.IsCompleteChecker additionalCompleteChecker) {
        return stickToWall(() -> this.update(desiredPositionToWall), additionalCompleteChecker);
    }

    public SequentialCommandSegment stickToWall(TeamElementFinder teamElementFinder, double requiredDistanceToWall, SequentialCommandSegment.IsCompleteChecker additionalCompleteChecker) {
        return stickToWall(
                () -> this.update(getDesiredAimingPositionToWall(teamElementFinder.teamElementPosition, requiredDistanceToWall)),
                additionalCompleteChecker
        );
    }

    public SequentialCommandSegment stickToWall(TeamElementFinder teamElementFinder, SequentialCommandSegment.IsCompleteChecker additionalCompleteChecker) {
        return stickToWall(
                () -> this.update(getDesiredAimingPositionToWall(teamElementFinder.teamElementPosition)),
                additionalCompleteChecker
        );
    }

    private SequentialCommandSegment stickToWall(Runnable updateCommand, SequentialCommandSegment.IsCompleteChecker additionalCompleteChecker) {
        return new SequentialCommandSegment(
                () -> true,
                () -> null,
                this::init,
                updateCommand,
                // () -> {},
                () -> chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY, new Vector2D()), modulesCommanderMarker),
                () -> !initSucceeded ||
                        (chassis.isCurrentTranslationalTaskRoughlyComplete() && chassis.isCurrentRotationalTaskRoughlyComplete() && additionalCompleteChecker.isComplete()),
                () -> new Rotation2D(0), () -> new Rotation2D(0)
        );
    }

    public Vector2D getDesiredAimingPositionToWall(TeamElementFinder.TeamElementPosition teamElementPosition) {
        return getDesiredAimingPositionToWall(teamElementPosition, desiredDistanceToWallSupplier.getAsDouble());
    }

    public Vector2D getDesiredAimingPositionToWall(TeamElementFinder.TeamElementPosition teamElementPosition, double requiredDistanceToWall) {
        double deviationFromCenter;
        switch (teamElementPosition) {
            case UNDETERMINED: case LEFT: {
                deviationFromCenter = autoStageScoringPositionsLeft;
                break;
            }
            case RIGHT: {
                deviationFromCenter = autoStageScoringPositionsRight;
                break;
            }
            case CENTER: {
                deviationFromCenter = autoStageScoringPositionsCenter;
                break;
            }
            default: throw new IllegalStateException("unknown team element result: " + teamElementPosition);
        }
        return new Vector2D(new double[] {
                RobotConfig.VisualNavigationConfigs.targetedRelativePositionToWallPreciseTOFApproach.getX() + deviationFromCenter,
                -requiredDistanceToWall
        });
    }

    boolean distanceSensorTrustable = true;
    boolean initSucceeded = false;

    private void init() {
        final long t0 = System.currentTimeMillis();
        while (!chassis.isVisualNavigationAvailable() && System.currentTimeMillis() - t0 < RobotConfig.VisualNavigationConfigs.maxTimeToWaitForVisualNavigationMS) {
            chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY, new Vector2D()), modulesCommanderMarker);
            chassis.periodic();
            aprilTagCamera.updateCamera();
            chassis.forceUpdateEncoders(modulesCommanderMarker);
            chassis.forceUpdateWheels(modulesCommanderMarker);
            try { Thread.sleep(50); } catch (InterruptedException ignored) {}
        }
        if (!chassis.isVisualNavigationAvailable()) {
            System.out.println("<-- warning: target not visible -->");
            initSucceeded = false;
            return;
        }
        final double distanceSensorReading = distanceSensor.getSensorReading();
        if (distanceSensorReading > RobotConfig.VisualNavigationConfigs.distanceSensorMaxDistance) {
            System.out.println("<-- warning: target might be too far -->");
            initSucceeded = false;
            return;
        }
        resetAimBot();
        updateWallPositionTOF();
        initSucceeded = true;
    }

    private void update(Vector2D desiredPositionToWall) {
        this.updateWallPositionTOF();
        chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                previousWallPosition.addBy(desiredPositionToWall)
        ), modulesCommanderMarker);
    }

    private void updateWallPositionTOF() {
        final double distanceSensorReading = distanceSensor.getSensorReading(),
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
