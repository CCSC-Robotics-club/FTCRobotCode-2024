package org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class TeamPropFinderColor {
    private final VisionPortal visionPortal;
    public TeamElementFinder.TeamElementPosition teamElementPosition = TeamElementFinder.TeamElementPosition.UNDETERMINED;
    public TeamPropFinderColor(WebcamName webcamName) {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(webcamName);
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        visionPortal = builder.build();
    }

    class TeamPropColorProcessor implements VisionProcessor {
        @Override
        public void init(int i, int i1, CameraCalibration cameraCalibration) {

        }

        @Override
        public Object processFrame(Mat mat, long l) {
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int i, int i1, float v, float v1, Object o) {

        }
    }

    public void findTeamElementOnce() {

    }
}
