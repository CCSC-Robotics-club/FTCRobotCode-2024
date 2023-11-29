package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FixedAnglePixelCamera;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;

public class PixelCameraAimBot {
    private final Chassis chassis;
    private final FixedAnglePixelCamera pixelCamera;
    private final ModulesCommanderMarker commanderMarker;
    private final TelemetrySender telemetrySender;
    private Vector2D previousPixelPosition = new Vector2D(new double[] {0, 0});


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
    private static final Vector2D feedingSweetSpot = new Vector2D(new double[] {0, -10}); // the robot's position to the pixel

    public PixelCameraAimBot(Chassis chassis, FixedAnglePixelCamera pixelCamera, ModulesCommanderMarker commanderMarker, TelemetrySender telemetrySender, Vector2D previousPixelPosition) {
        this.chassis = chassis;
        this.pixelCamera = pixelCamera;
        this.commanderMarker = commanderMarker;
        this.telemetrySender = telemetrySender;
        this.previousPixelPosition = previousPixelPosition;
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
        // TODO write this method
        return true;
    }
}
