package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FixedAnglePixelCamera;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;

import java.util.Map;

public class PixelCameraAimBot {
    private final Chassis chassis;
    private final FixedAnglePixelCamera pixelCamera;
    private final ModulesCommanderMarker commanderMarker;
    private final Map<String, Object> debugMessages;
    private Vector2D pixelFieldPosition = null;

    private long searchInitateTimeMillis = -1;
    private long searchUntilTimeMillis = -1;
    private Vector2D searchStartPosition = null;
    private Vector2D searchDirection = null;
    public enum AimMethod {
        FACE_TO_AND_FEED, // the robot rotates to face the targeted pixel
        LINE_UP_AND_FEED // the robot moves horizontally to line up with the targeted pixel
    }
    private enum Status {
        UNUSED,
        SEARCHING,
        LINING_UP,
        FACING_TO,
        FEEDING
    }
    private Status status;


    public PixelCameraAimBot(Chassis chassis, FixedAnglePixelCamera pixelCamera, ModulesCommanderMarker commanderMarker, Map<String, Object> debugMessages) {
        this.chassis = chassis;
        this.pixelCamera = pixelCamera;
        this.commanderMarker = commanderMarker;
        this.debugMessages = debugMessages;

        status = Status.UNUSED;
    }

    public SequentialCommandSegment createAimingCommandSegment(AimMethod aimMethod) {
        return new SequentialCommandSegment(
                () -> true,
                null,
                () -> initiateAim(aimMethod),
                this::update,
                () -> chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY, new Vector2D()), commanderMarker),
                () -> status==Status.UNUSED,
                chassis::getRotation, chassis::getRotation
        );
    }

    /**
     * searches for the target by moving horizontally for a distance
     * @param searchRangeCM the amount of centimeters to search for, negative is to the left
     * @param robotFacingRotation the robotFacingRotation to face
     * */
    public SequentialCommandSegment createSearchAndAimCommandSegment(double searchRangeCM, double robotFacingRotation) {
        return new SequentialCommandSegment(
                () -> true,
                null,
                () -> initiateSearch(searchRangeCM, robotFacingRotation),
                this::update,
                () -> chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY, new Vector2D()), commanderMarker),
                () -> status == Status.UNUSED,
                () -> new Rotation2D(robotFacingRotation), () -> new Rotation2D(robotFacingRotation)
        );
    }

    public void initiateSearch(double searchRangeCM, double robotFacingRotation) {
        this.searchDirection = new Vector2D(robotFacingRotation - Math.PI / 2, RobotConfig.VisualNavigationConfigs.pixelSearchVelocity);
        this.searchInitateTimeMillis = System.currentTimeMillis();
        this.searchUntilTimeMillis = (long) ((searchRangeCM / RobotConfig.VisualNavigationConfigs.pixelSearchVelocity) * 1000.0f + this.searchInitateTimeMillis);
                this.status = Status.SEARCHING;
    }

    /**
     * @return whether the aim task is initiated or denied because the target is lost
     * */
    private double startingRotation = 0;
    public boolean initiateAim(AimMethod aimMethod) {
        final Vector2D targetFieldPosition = getTargetFieldPosition();
        if (targetFieldPosition == null)
            return false;

        this.status = aimMethod == AimMethod.FACE_TO_AND_FEED ? Status.FACING_TO : Status.LINING_UP;
        this.pixelFieldPosition = targetFieldPosition;
        startingRotation = chassis.getYaw();
        return true;
    }

    public void cancel() {
        this.status = Status.UNUSED;
        // pixelCamera.disableCamera();
    }

    public void update() {
        // pixelCamera.enableCamera();
        debugMessages.put("pixel aim-bot status", status);
        debugMessages.put("camera target", pixelCamera.getNearestPixelPosition());
        switch (status) {
            case FACING_TO: {
                // TODO bugs are found in this aiming mode, the robot goes to the wrong rotation
                updateTargetPositionIfSeen();
                double targetedRotation = pixelFieldPosition.getHeading() + Math.PI / 2;
                chassis.setRotationalTask(
                        new Chassis.ChassisRotationalTask(
                                Chassis.ChassisRotationalTask.ChassisRotationalTaskType.GO_TO_ROTATION,
                                targetedRotation),
                        commanderMarker
                );
                final Vector2D desiredFieldPosition = pixelFieldPosition.addBy(
                        RobotConfig.VisualNavigationConfigs.pixelFeedingSweetSpot.multiplyBy(new Rotation2D(targetedRotation))
                );
                chassis.setTranslationalTask(
                        new Chassis.ChassisTranslationalTask(
                                Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                                desiredFieldPosition
                        ), commanderMarker);

                debugMessages.put("pixel direction", targetedRotation);
                debugMessages.put("face-to targeted position", desiredFieldPosition);

                if (chassis.isCurrentRotationalTaskComplete())
                    initiateFeed();
                return;
            }
            case LINING_UP: {
                updateTargetPositionIfSeen();
                final Vector2D desiredFieldPosition = pixelFieldPosition.addBy(
                                RobotConfig.VisualNavigationConfigs.pixelFeedingSweetSpot.multiplyBy(new Rotation2D(chassis.getYaw()))
                        );
                debugMessages.put("line-up targeted position", desiredFieldPosition);
                chassis.setTranslationalTask(
                        new Chassis.ChassisTranslationalTask(
                                Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                                desiredFieldPosition
                        ), commanderMarker);
                chassis.setRotationalTask(
                        new Chassis.ChassisRotationalTask(
                                Chassis.ChassisRotationalTask.ChassisRotationalTaskType.GO_TO_ROTATION,
                                startingRotation),
                        commanderMarker
                );

                if (Vector2D.displacementToTarget(chassis.getChassisEncoderPosition(), desiredFieldPosition).getMagnitude() < RobotConfig.VisualNavigationConfigs.feedingSpotErrorTolerance)
                    initiateFeed();
                return;
            }
            case FEEDING: {
                if (chassis.isCurrentTranslationalTaskComplete()
                        || System.currentTimeMillis()-feedingProcessStartTimeMillis >= RobotConfig.VisualNavigationConfigs.feedTimeMillis)
                    status = Status.UNUSED;
                return;
            }
            case SEARCHING: {
                if (initiateAim(AimMethod.LINE_UP_AND_FEED))
                    return;
                if (System.currentTimeMillis() > searchUntilTimeMillis) {
                    this.status = Status.UNUSED;
                    return;
                }

                final Vector2D desiredSearchPosition = this.searchStartPosition.addBy(
                        searchDirection.multiplyBy(
                                (System.currentTimeMillis() - searchInitateTimeMillis) / 1000.f
                        ));

                chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                        Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                        desiredSearchPosition
                ), commanderMarker);
            }
        }
    }

    private double feedingProcessStartTimeMillis = 0;
    private void initiateFeed() {
        final Vector2D feedStartPosition = chassis.getChassisEncoderPosition(),
                feedPathForward = new Vector2D(new double[] {0, RobotConfig.VisualNavigationConfigs.feedingDistanceForward}),
                feedEndPosition = feedStartPosition.addBy(feedPathForward.multiplyBy(new Rotation2D(chassis.getYaw())));

        debugMessages.put("feed end pos", feedEndPosition);

        chassis.setTranslationalTask(
                new Chassis.ChassisTranslationalTask(
                        Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                        feedEndPosition), commanderMarker);
        this.status = Status.FEEDING;
        feedingProcessStartTimeMillis = System.currentTimeMillis();
    }

    private void updateTargetPositionIfSeen() {
        final Vector2D pixelFieldPositionNew = getTargetFieldPosition();

        debugMessages.put("pixel position update", pixelFieldPositionNew);

        if (pixelFieldPositionNew != null)
            pixelFieldPosition = pixelFieldPositionNew;
    }

    private Vector2D getTargetFieldPosition() {
        Vector2D targetRelativePositionToRobot = pixelCamera.getNearestPixelPosition();
        if (targetRelativePositionToRobot == null) return null;
        return chassis.getChassisEncoderPosition().addBy(
                targetRelativePositionToRobot.multiplyBy(new Rotation2D(chassis.getYaw()))
        );
    }

    public boolean shouldIntakeStart() {
        return this.status == Status.FEEDING;
    }

    public boolean isAimBotBusy() {
        return this.status != Status.UNUSED;
    }
}
