package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.RobotConfig;

public class AprilTagCameraAndDistanceSensorAimBot {
    private final Chassis chassis;
    private final DistanceSensor distanceSensor;
    private final FixedAngleArilTagCamera aprilTagCamera;
    private final ModulesCommanderMarker modulesCommanderMarker;
    private Vector2D previousWallPosition = new Vector2D(0, Double.POSITIVE_INFINITY);
    private double previousWallDistance = Double.POSITIVE_INFINITY;
    public AprilTagCameraAndDistanceSensorAimBot(Chassis chassis, DistanceSensor distanceSensor, FixedAngleArilTagCamera aprilTagCamera, ModulesCommanderMarker commanderMarker) {
        this.chassis = chassis;
        this.distanceSensor = distanceSensor;
        this.aprilTagCamera = aprilTagCamera;
        this.modulesCommanderMarker = commanderMarker;
    }

    public SequentialCommandSegment createCommandSegment(Vector2D desiredPositionToWall) {
        return new SequentialCommandSegment(
                null,
                this::init,
                () -> this.update(desiredPositionToWall),
                () -> chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY, new Vector2D()), modulesCommanderMarker),
                chassis::isCurrentTranslationalTaskComplete,
                0, 0
        );
    }

    boolean distanceSensorTrustable = true;

    private void init() {
        final long t0 = System.currentTimeMillis();
        while (!chassis.isVisualNavigationAvailable() && System.currentTimeMillis() - t0 < RobotConfig.VisualNavigationConfigs.maxTimeToWaitForVisualNavigationMS) {
            chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY, new Vector2D()), modulesCommanderMarker);
            chassis.periodic();
            chassis.forceUpdateCamera(modulesCommanderMarker);
            chassis.forceUpdateEncoders(modulesCommanderMarker);
            chassis.forceUpdateWheels(modulesCommanderMarker);
            try { Thread.sleep(50); } catch (InterruptedException ignored) {}
            // throw new IllegalStateException("waiting for target");
        }
        if (!chassis.isVisualNavigationAvailable()) throw new IllegalStateException("don't see the wall after " + RobotConfig.VisualNavigationConfigs.maxTimeToWaitForVisualNavigationMS + "ms");
        final double distanceSensorReading = distanceSensor.getDistance(DistanceUnit.CM);
        if (distanceSensorReading > RobotConfig.VisualNavigationConfigs.distanceSensorMaxDistance) throw new IllegalStateException("target too far");
        resetAimBot();
        updateWallPositionTOF();
    }

    private void update(Vector2D desiredPositionToWall) {
        this.updateWallPositionTOF();
        chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                previousWallPosition.addBy(desiredPositionToWall)
        ), null);
    }

    private void updateWallPositionTOF() {
        final double distanceSensorReading = distanceSensor.getDistance(DistanceUnit.CM),
                newWallPositionX = chassis.isVisualNavigationAvailable() ?
                        chassis.getChassisEncoderPosition().getX() - chassis.getRelativeFieldPositionToWall().getX():
                        previousWallPosition.getX();
        final Vector2D newWallPosition = new Vector2D(
                new double[]{
                        newWallPositionX,
                        chassis.getChassisEncoderPosition().getY() + distanceSensorReading
                });

        if (distanceSensorTrustable && Vector2D.displacementToTarget(previousWallPosition, newWallPosition).getMagnitude() > RobotConfig.VisualNavigationConfigs.errorTolerance / 2)
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
