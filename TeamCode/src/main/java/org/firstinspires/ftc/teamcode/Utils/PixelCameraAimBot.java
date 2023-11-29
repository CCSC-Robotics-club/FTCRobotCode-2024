package org.firstinspires.ftc.teamcode.Utils;

import android.text.Spannable;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FixedAnglePixelCamera;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;

public class PixelCameraAimBot {
    private final Chassis chassis;
    private final FixedAnglePixelCamera pixelCamera;
    private final ModulesCommanderMarker commanderMarker;
    private final TelemetrySender telemetrySender;
    private Vector2D previousPixelPosition = null;


    public enum AimMethod {
        FACE_TO_AND_FEED, // the robot rotates to face the targeted pixel
        LINE_UP_AND_FEED // the robot moves horizontally to line up with the targeted pixel
    }
    private enum Status {
        UNUSED,
        LINING_UP,
        FACING_TO,
        FEEDING
    }
    private Status status;

    private static final Vector2D feedingSweetSpot = new Vector2D(new double[] {0, -10}); // the robot's position to the pixel

    public PixelCameraAimBot(Chassis chassis, FixedAnglePixelCamera pixelCamera, ModulesCommanderMarker commanderMarker, TelemetrySender telemetrySender) {
        this.chassis = chassis;
        this.pixelCamera = pixelCamera;
        this.commanderMarker = commanderMarker;
        this.telemetrySender = telemetrySender;

        status = Status.UNUSED;
    }

    public SequentialCommandSegment createAimingCommandSegment(AimMethod aimMethod) {
        return null; // TODO write this
    }

    /**
     * searches for the target by moving horizontally for a distance
     * @param searchRangeCM the amount of centimeters to search for, negative is to the left
     * */
    public SequentialCommandSegment createSearchAndAimCommandSegment(double searchRangeCM) {
        return null; // TODO write this command
    }

    /**
     * @return whether the aim task is initiated or denied because the target is lost
     * */
    public boolean initiateAim(AimMethod aimMethod) {
        final Vector2D targetFieldPosition = getTargetFieldPosition();
        if (targetFieldPosition == null)
            return false;

        this.status = aimMethod == AimMethod.FACE_TO_AND_FEED ? Status.FACING_TO : Status.LINING_UP;
        this.previousPixelPosition = targetFieldPosition;
        return true;
    }

    public void update() {
        switch (status) {
            case FACING_TO: {
                updateTargetPositionIfSeen();
                double targetedRotation = previousPixelPosition.getHeading() - Math.PI / 2;
                chassis.setRotationalTask(
                        new Chassis.ChassisRotationalTask(
                                Chassis.ChassisRotationalTask.ChassisRotationalTaskType.GO_TO_ROTATION,
                                targetedRotation),
                        commanderMarker
                );

                if (chassis.isCurrentRotationalTaskComplete())
                    this.status = Status.FEEDING;
                return;
            }
            case LINING_UP: {
                updateTargetPositionIfSeen();
                double verticalDistanceToRobot = previousPixelPosition.
                        addBy(
                                chassis.getChassisEncoderPosition().multiplyBy(-1))
                        .multiplyBy(new Rotation2D(chassis.getYaw()).getReversal()
                        ).getY();
                final Vector2D desiredRelativePositionToRobot = new Vector2D(new double[] {0, verticalDistanceToRobot}),
                        desiredFieldPosition = chassis.getChassisEncoderPosition().addBy(
                                desiredRelativePositionToRobot.multiplyBy(new Rotation2D(chassis.getYaw()))
                        );
                chassis.setTranslationalTask(
                        new Chassis.ChassisTranslationalTask(
                                Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                                desiredFieldPosition
                        ), commanderMarker);

                if (chassis.isCurrentTranslationalTaskComplete())
                    this.status = Status.FEEDING;
                return;
            }
            case FEEDING: {
                updateTargetPositionIfSeen();

                final Vector2D pixelPositionToRobot = previousPixelPosition.
                        addBy(
                                chassis.getChassisEncoderPosition().multiplyBy(-1))
                        .multiplyBy(new Rotation2D(chassis.getYaw()).getReversal()
                );

                if (pixelPositionToRobot.getY() < 0)
                    status = Status.UNUSED;

                final Vector2D desiredPositionToRobot = new Vector2D(new double[]
                        {pixelPositionToRobot.getX(), RobotConfig.ChassisConfigs.targetDistanceAtMaxDesiredSpeed * 0.4});
                chassis.setTranslationalTask(
                        new Chassis.ChassisTranslationalTask(
                                Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                                desiredPositionToRobot.multiplyBy(new Rotation2D(chassis.getYaw()))
                                ),
                        commanderMarker
                );
            }
        }
    }

    private void updateTargetPositionIfSeen() {
        final Vector2D targetFieldPosition = getTargetFieldPosition();
        if (targetFieldPosition != null)
            previousPixelPosition = targetFieldPosition;
    }

    private Vector2D getTargetFieldPosition() {
        Vector2D targetRelativePositionToRobot = pixelCamera.getNearestPixelPosition();
        if (targetRelativePositionToRobot == null) return null;
        return chassis.getChassisEncoderPosition().addBy(
                targetRelativePositionToRobot.multiplyBy(new Rotation2D(chassis.getYaw()))
        );
    }
}
