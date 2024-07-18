package org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * this is a drop-in replacement for the tensorflow version of team element finder, which uses color detection
 */
public class TeamElementFinderColor {
    private static final int width = 320, height = 240;
    private final RectangularRegionColorComparisonPipeLine.RegionOfInterest
            LEFT = new RectangularRegionColorComparisonPipeLine.RegionOfInterest(50, 50, 160-80, 120),
            CENTER = new RectangularRegionColorComparisonPipeLine.RegionOfInterest(50, 50, 160, 60),
            RIGHT = new RectangularRegionColorComparisonPipeLine.RegionOfInterest(50, 50, 160+80, 120);

    private final RectangularRegionColorComparisonPipeLine.RegionOfInterest[] ROIs = new RectangularRegionColorComparisonPipeLine.RegionOfInterest[] {
            LEFT, CENTER, RIGHT
    };
    private final Gamepad gamepad;
    private final RectangularRegionColorComparisonPipeLine pipeLine;
    private int currentRIOPositionTuningIndex = 0;


    public TeamElementFinderColor(HardwareMap hardwareMap, WebcamName webcamName, Gamepad gamepad, Telemetry telemetry, Robot.Side allianceSide) {
        this.gamepad = gamepad;

        final int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );
        final OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(
                webcamName,
                cameraMonitorViewId
        );

        webcam.setPipeline(this.pipeLine = new RectangularRegionColorComparisonPipeLine(
                allianceSide == Robot.Side.RED ?
                        RectangularRegionColorComparisonPipeLine.ColorChannel.RED
                        :RectangularRegionColorComparisonPipeLine.ColorChannel.BLUE,
                telemetry,
                ROIs
        ));

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 5);
            }
            @Override
            public void onError(int errorCode) {
                throw new RuntimeException("Error while opening camera, code: " + errorCode);
            }
        });
    }

    /**
     * call this method 20 times a scond before the start of match
     * gamepad can be used to tune the positions of the Regions of Interest
     * */
    public void beforeStartPeriodic() {
        if (gamepad.x)
            currentRIOPositionTuningIndex = 0;
        else if (gamepad.y)
            currentRIOPositionTuningIndex = 1;
        else if (gamepad.b)
            currentRIOPositionTuningIndex = 2;

        final RectangularRegionColorComparisonPipeLine.RegionOfInterest ROI = ROIs[currentRIOPositionTuningIndex];
        ROI.centerX += gamepad.left_stick_x;
        ROI.centerX = Math.min(Math.max(ROI.centerX, 0), width);
        ROI.centerY += gamepad.left_stick_y;
        ROI.centerY = Math.min(Math.max(ROI.centerY, 0), height);
    }

    public RobotConfig.TeamElementPosition getBestResult() {
        RectangularRegionColorComparisonPipeLine.RegionOfInterest result = this.pipeLine.getRegionWithHighestDensity();
        if (result == LEFT) {
            return RobotConfig.TeamElementPosition.LEFT;
        } else if (result == CENTER) {
            return RobotConfig.TeamElementPosition.CENTER;
        } else if (result == RIGHT) {
            return RobotConfig.TeamElementPosition.RIGHT;
        }
        throw new IllegalStateException("pipe line result " + result + " is not one of LEFT, CENTER or RIGHT region");
    }
}
