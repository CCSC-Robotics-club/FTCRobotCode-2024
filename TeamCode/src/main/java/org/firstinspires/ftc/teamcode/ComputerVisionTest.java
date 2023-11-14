package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Point;

import java.util.List;


public class ComputerVisionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor myAprilTagProcessor;
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "testcam1"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        waitForStart();
        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("frame rate", myVisionPortal.getFps());

            List<AprilTagDetection> detections = myAprilTagProcessor.getDetections();
            for (AprilTagDetection detection:detections) {
                if (detection.metadata == null) continue;
                telemetry.addData("object detected, id:", detection.id);
                telemetry.addData("position(x)", (int)detection.center.x);
                telemetry.addData("position(y)", (int)detection.center.y);

                telemetry.addData("x", detection.rawPose.x);
                telemetry.addData("y", detection.rawPose.y);
                telemetry.addData("z", detection.rawPose.z);
                telemetry.addData("R", detection.rawPose.R);
                telemetry.addLine();
            }

            telemetry.update();
        }

        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, false);
    }

//    private double getSize(Point[] corners) {
//        double upperSideLength = corners[1]
//    }
}
