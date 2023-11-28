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
    private long previousTimeMillis = -1;
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
                () -> {
                    final long t0 = System.currentTimeMillis();
                    while (!chassis.isVisualNavigationAvailable() && System.currentTimeMillis() - t0 < RobotConfig.VisualNavigationConfigs.maxTimeToWaitForVisualNavigationMS) {
                        chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY, new Vector2D()), modulesCommanderMarker);
                        chassis.periodic();
                        try { Thread.sleep(50); } catch (InterruptedException ignored) {}
                    }
                    final double distanceSensorReading = distanceSensor.getDistance(DistanceUnit.CM);
                    if (distanceSensorReading > RobotConfig.VisualNavigationConfigs.distanceSensorMaxDistance) throw new IllegalStateException("target too far");
                    if (!chassis.isVisualNavigationAvailable()) throw new IllegalStateException("don't see the wall after " + RobotConfig.VisualNavigationConfigs.maxTimeToWaitForVisualNavigationMS + "ms");
                    resetAimBot();
                    updateWallPositionTOF(0);
                    },
                () -> {
                    double dt = (System.currentTimeMillis() - previousTimeMillis) / 1000.0f;
                    this.updateWallPositionTOF(dt);

                    previousTimeMillis = System.currentTimeMillis();
                },
                () -> {

                },
                () -> true,
                0, 0
        );
    }

    boolean distanceSensorTrustable = true;
    private void updateWallPositionTOF(double dt) {
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

        /* to see if the distance sensor is failing (sometimes, the robot shifts too much to the side and the distance sensor are not scanning the target) */
        timeSinceLastWallVelocityUpdate += dt;
        if (timeSinceLastWallVelocityUpdate < RobotConfig.VisualNavigationConfigs.timeRevealExamineDistanceSensorValidity)
            return;
        double change = distanceSensorReading - previousWallDistance;
        if (change / timeSinceLastWallVelocityUpdate > RobotConfig.VisualNavigationConfigs.approachReverseSpeedTolerance)
            distanceSensorTrustable = false;
        timeSinceLastWallVelocityUpdate = 0;
    }

    private void resetAimBot() {
        previousTimeMillis = System.currentTimeMillis();
        distanceSensorTrustable = true;
    }
}
