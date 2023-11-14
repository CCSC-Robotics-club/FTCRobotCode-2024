package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.Utils.FixedAngleCameraProfile;
import org.firstinspires.ftc.teamcode.Utils.RawPixelDetectionCamera;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class FixedAnglePixelCamera extends RobotModule {
    private final RawPixelDetectionCamera camera;
    private final FixedAngleCameraProfile cameraProfile;
    public FixedAnglePixelCamera(RawPixelDetectionCamera camera, FixedAngleCameraProfile profile) {
        super("PixelCamera", 24);
        this.camera = camera;
        this.cameraProfile = profile;
    }

    public void enableCamera() {
        camera.startRecognizing();
    }

    public void disableCamera() {
        camera.stopRecognizing();
    }

    public Vector2D getNearestPixelPosition() {
        List<RawPixelDetectionCamera.PixelTargetRaw> targetsRaw = new ArrayList<>(1);
        if (targetsRaw.isEmpty())
            return null;
        Vector2D pixelPosition = new Vector2D(0, Double.POSITIVE_INFINITY);
        for (RawPixelDetectionCamera.PixelTargetRaw targetRaw:targetsRaw) {
            Vector2D pixelPositionNew = calculatePosition(targetRaw.x, targetRaw.y);
            if (pixelPositionNew.getMagnitude() < pixelPosition.getMagnitude())
                pixelPosition = pixelPositionNew;
        }
        return pixelPosition;
    }

    private Vector2D calculatePosition(double pixelX, double pixelY) {
        final double yDistance = cameraProfile.getDistanceFromYPixel(pixelY),
                xDistance = yDistance / Math.tan(cameraProfile.getTargetAngleRadianFromXPixel(pixelX));
        return new Vector2D(new double[] {xDistance, yDistance});
    }

    @Override
    public void init() {

    }

    @Override
    protected void periodic(double dt) {

    }

    @Override
    protected void onDestroy() {

    }

    @Override
    public void reset() {

    }
}
