package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.FixedAngleCameraProfile;
import org.firstinspires.ftc.teamcode.Utils.RawPixelDetectionCamera;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.Transformation2D;
import org.firstinspires.ftc.teamcode.Utils.Vector2D;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FixedAnglePixelCamera extends RobotModule {
    private final RawPixelDetectionCamera camera;
    private final FixedAngleCameraProfile cameraProfile;
    private final Rotation2D cameraFacingCalibration;

    private List<RawPixelDetectionCamera.PixelTargetRaw> targetsRaw = new ArrayList<>(1);
    private final Map<String, Object> debugMessages = new HashMap<>(1);
    public FixedAnglePixelCamera(RawPixelDetectionCamera camera, FixedAngleCameraProfile profile, double cameraFacingDirection) {
        super("PixelCamera", 24);
        this.camera = camera;
        this.cameraProfile = profile;
        this.cameraFacingCalibration = new Rotation2D(cameraFacingDirection);
    }

    public void enableCamera() {
        camera.startRecognizing();
    }

    public void disableCamera() {
        camera.stopRecognizing();
    }

    /**
     * get the relative position of the nearest pixel target to camera
     * ignore any target too far (VisualNavigationConfigs.pixelDetectionMaximumDistance)
     * @return the position, in vector and in cm, null for unseen
     * */
    public Vector2D getNearestPixelPosition() {
        Vector2D pixelPosition = new Vector2D(Math.PI/2, RobotConfig.VisualNavigationConfigs.pixelDetectionMaximumDistance);
        for (RawPixelDetectionCamera.PixelTargetRaw targetRaw:targetsRaw) {
            Vector2D pixelPositionNew = calculatePosition(targetRaw.x, targetRaw.y);
            if (pixelPositionNew.getMagnitude() < pixelPosition.getMagnitude())
                pixelPosition = pixelPositionNew;
        }
        return pixelPosition.getMagnitude() < RobotConfig.VisualNavigationConfigs.pixelDetectionMaximumDistance ?
                pixelPosition.multiplyBy(cameraFacingCalibration) : null;
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
        targetsRaw = camera.getPixelTargets();
        debugMessages.put("raw pixel target", !camera.getPixelTargets().isEmpty() ? camera.getPixelTargets().get(0) : "unseen");
    }

    @Override
    protected void onDestroy() {

    }

    @Override
    public void reset() {

    }

    @Override
    public Map<String, Object> getDebugMessages() {
        return debugMessages;
    }
}
