package org.firstinspires.ftc.teamcode.Utils;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

public class TensorCamera implements RawPixelDetectionCamera {
    private static final Size defaultCameraResolution = new Size(640, 480);
    private static final int[] defaultLeftRightCast = new int[] {0, 0}, defaultTopBottomCast = {0, 0};
    private static float minConfident = 0.75f;
    private final TfodProcessor tfod;
    private final int[] cameraResolution;
    private final VisionPortal.Builder visionPortalBuilding;
    private final VisionPortal visionPortal;
    public TensorCamera(WebcamName camera) {
        this(camera, defaultCameraResolution);
    }

    public TensorCamera(WebcamName camera, Size cameraResolution) {
        this(camera, cameraResolution, defaultLeftRightCast, defaultTopBottomCast);
    }

    public TensorCamera(WebcamName camera, Size cameraResolution, int[] leftRightCast, int[] topBottomCast) {
        this.tfod = TfodProcessor.easyCreateWithDefaults();
        this.tfod.setClippingMargins(leftRightCast[0], topBottomCast[0], leftRightCast[1], topBottomCast[1]);
        this.tfod.setMinResultConfidence(minConfident);

        visionPortalBuilding = new VisionPortal.Builder();
        visionPortalBuilding.setCamera(camera);
        visionPortalBuilding.setCameraResolution(cameraResolution);
        this.cameraResolution = new int[] {cameraResolution.getWidth(), cameraResolution.getHeight()};
        visionPortalBuilding.enableLiveView(true);
        visionPortalBuilding.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        visionPortalBuilding.addProcessor(tfod);

        this.visionPortal = visionPortalBuilding.build();

        startRecognizing();
    }

    @Override
    public void startRecognizing() {
        visionPortal.resumeStreaming();
    }

    /** saves cpu resource */
    @Override
    public void stopRecognizing() {
        visionPortal.stopStreaming();
    }

    @Override
    public List<PixelTargetRaw> getPixelTargets() {
        final List<PixelTargetRaw> pixelTargets = new ArrayList<>(1);
        for (Recognition recognition : tfod.getRecognitions()) {
            if (!recognition.getLabel().equals("Pixel"))
                continue;
            double x = (recognition.getLeft()+recognition.getRight())/2 - cameraResolution[0] / 2f;
            double y = (recognition.getTop()+recognition.getBottom())/2 - cameraResolution[1] / 2f;

            pixelTargets.add(new PixelTargetRaw(x, y));
        }
        return pixelTargets;
    }

    public double getCameraRefreshRate() {
        return visionPortal.getFps();
    }
}
