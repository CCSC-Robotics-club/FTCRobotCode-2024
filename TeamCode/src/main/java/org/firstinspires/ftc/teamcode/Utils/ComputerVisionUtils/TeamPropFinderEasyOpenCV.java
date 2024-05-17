package org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class TeamPropFinderEasyOpenCV {
    private final WebcamName webcamName;
    private final OpenCvWebcam webcam;
    public TeamElementFinder.TeamElementPosition teamElementPosition = TeamElementFinder.TeamElementPosition.UNDETERMINED;
    public TeamPropFinderEasyOpenCV(WebcamName webcamName) {
        this.webcamName = webcamName;

        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        this.webcam.setPipeline(new TeamPropColorDetectionPipeLine());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480);
            }
            @Override
            public void onError(int errorCode) {

            }
        });
    }

    class TeamPropColorDetectionPipeLine extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat mat) {
            return null;
        }
    }

    public void findTeamElementOnce() {

    }
}
