package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.FixedAngleCameraProfile;
import org.firstinspires.ftc.teamcode.Utils.RawPixelDetectionCamera;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.Vector2D;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FixedAnglePixelCamera extends RobotModule {
    private final RawPixelDetectionCamera camera;
    private final FixedAngleCameraProfile cameraProfile;

    private List<RawPixelDetectionCamera.PixelTargetRaw> targetsRaw = new ArrayList<>(1);
    private final Map<String, Object> debugMessages = new HashMap<>(1);
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

    /**
     * get the relative position of the nearest pixel target to camera
     * ignore any target too far (VisualNavigationConfigs.pixelDetectionMaximumDistance)
     * @return the position, in vector and in cm, null for unseen
     * */
    public Vector2D getNearestPixelPosition() {
        super.periodic();
        Vector2D pixelPosition = new Vector2D(Math.PI/2, RobotConfig.VisualNavigationConfigs.pixelDetectionMaximumDistance);
        for (RawPixelDetectionCamera.PixelTargetRaw targetRaw:targetsRaw) {
            Vector2D pixelPositionNew = calculatePosition(targetRaw.x, targetRaw.y);
            if (pixelPositionNew.getMagnitude() < pixelPosition.getMagnitude())
                pixelPosition = pixelPositionNew;
        }
        return pixelPosition.getMagnitude() < RobotConfig.VisualNavigationConfigs.pixelDetectionMaximumDistance ? pixelPosition : null;
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
