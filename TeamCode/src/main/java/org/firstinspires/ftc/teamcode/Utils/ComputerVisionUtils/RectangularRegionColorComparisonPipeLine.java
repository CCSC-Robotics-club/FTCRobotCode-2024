// Created By Iron Maple (19581) and ChatGPT, Under MIT-License
// original source https://github.com/Shenzhen-Robotics-Alliance/Maple-FTC-Skeleton-2024
package org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RectangularRegionColorComparisonPipeLine extends OpenCvPipeline {
    public enum ColorChannel {
        RED,
        BLUE
    }

    private final RegionOfInterest[] regionOfInterests;
    private final ColorChannel focusedColorChannel;
    private final Telemetry telemetry;

    private RegionOfInterest regionWithHighestDensity;

    public RectangularRegionColorComparisonPipeLine(ColorChannel focusedColorChannel, Telemetry telemetry, RegionOfInterest... regionOfInterests) {
        this.focusedColorChannel = focusedColorChannel;
        this.telemetry = telemetry;
        this.regionOfInterests = regionOfInterests;

        if (regionOfInterests.length < 1)
            throw new IllegalArgumentException("there has to be at least one ROI!");
    }

    private static final Scalar YELLOW = new Scalar(255, 255, 0);
    private static final Scalar GREEN = new Scalar(0, 255, 0);
    private final Mat YCrCbImage = new Mat();
    private final Mat focusedChannelMat = new Mat();
    private final Mat streamingImage = new Mat();

    /**
     * Called 24 times a second, when an image is captured from the camera
     * Processes the results from the input and draws marks on the output
     * @param input the latest image from the camera, RGB
     * @return the processed image with marks (such as bounding-boxes) on it, the returned image will be streamed to the Driver-Station
     */
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, YCrCbImage, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCbImage, focusedChannelMat, focusedColorChannel == ColorChannel.RED ? 1 : 2);

        toRGBImageGray(focusedChannelMat, streamingImage);

        double highestDensity = Double.NEGATIVE_INFINITY;

        int regionID = 0;
        for (RegionOfInterest regionOfInterest : regionOfInterests) {
            double density = getFocusedChannelDensityInRegion(focusedChannelMat, regionOfInterest);
            telemetry.addData("density of ROI" + regionID, density);
            telemetry.addData("center of ROI" + regionID++, regionOfInterest.centerX + ", " + regionOfInterest.centerY);
            if (density > highestDensity) {
                highestDensity = density;
                regionWithHighestDensity = regionOfInterest;
            }
        }

        for (RegionOfInterest regionOfInterest : regionOfInterests) {
            if (regionOfInterest.equals(regionWithHighestDensity)) {
                fillRegion(streamingImage, regionOfInterest, GREEN);
            } else {
                drawRegion(streamingImage, regionOfInterest);
            }
        }

        telemetry.update();
        return streamingImage;
    }

    /**
     * Compares the density of the focused color channel of each ROI
     * The result is cached from the latest call to processFrame()
     * @return the ROI with the highest pixel density of the focused color channel
     */
    public RegionOfInterest getRegionWithHighestDensity() {
        return regionWithHighestDensity;
    }

    private static final Mat redChannel = new Mat(), greenChannel = new Mat(), blueChannel = new Mat();

    private static void toRGBImageGray(Mat focusedChannelMat, Mat targetedMat) {
        // Copy the grayscale values into each channel
        focusedChannelMat.copyTo(redChannel);
        focusedChannelMat.copyTo(greenChannel);
        focusedChannelMat.copyTo(blueChannel);

        // Merge the channels to form the RGB image
        List<Mat> channels = new ArrayList<>();
        channels.add(blueChannel);  // Blue
        channels.add(greenChannel); // Green
        channels.add(redChannel);   // Red

        Core.merge(channels, targetedMat);
    }

    private static void drawRegion(Mat imageToDrawOn, RegionOfInterest regionOfInterest) {
        Point topLeft = new Point(
                regionOfInterest.centerX - regionOfInterest.sizeX / 2,
                regionOfInterest.centerY - regionOfInterest.sizeY / 2
        );
        Point bottomRight = new Point(
                regionOfInterest.centerX + regionOfInterest.sizeX / 2,
                regionOfInterest.centerY + regionOfInterest.sizeY / 2
        );

        // Draw the rectangle
        Imgproc.rectangle(imageToDrawOn, topLeft, bottomRight, YELLOW, 2);
    }

    private static void fillRegion(Mat imageToDrawOn, RegionOfInterest regionOfInterest, Scalar color) {
        Point topLeft = new Point(
                regionOfInterest.centerX - regionOfInterest.sizeX / 2,
                regionOfInterest.centerY - regionOfInterest.sizeY / 2
        );
        Point bottomRight = new Point(
                regionOfInterest.centerX + regionOfInterest.sizeX / 2,
                regionOfInterest.centerY + regionOfInterest.sizeY / 2
        );

        // Draw the filled rectangle
        Imgproc.rectangle(imageToDrawOn, topLeft, bottomRight, color, Imgproc.FILLED);
    }

    /**
     * @param focusedChannelMat the mat of the focused channel of the camera image, such as Cr or Cb
     * @param regionOfInterest the region to calculate
     * @return the density of the color of the focused channel over the region (density = sum of channel value / area)
     */
    private static double getFocusedChannelDensityInRegion(Mat focusedChannelMat, RegionOfInterest regionOfInterest) {
        // Calculate the top-left and bottom-right points of the rectangle
        int startX = (int) (regionOfInterest.centerX - regionOfInterest.sizeX / 2);
        int startY = (int) (regionOfInterest.centerY - regionOfInterest.sizeY / 2);
        int endX = (int) (regionOfInterest.centerX + regionOfInterest.sizeX / 2);
        int endY = (int) (regionOfInterest.centerY + regionOfInterest.sizeY / 2);

        // Ensure the ROI is within the bounds of the image
        startX = Math.max(0, startX);
        startY = Math.max(0, startY);
        endX = Math.min(focusedChannelMat.cols(), endX);
        endY = Math.min(focusedChannelMat.rows(), endY);

        // Define the ROI
        Rect roi = new Rect(startX, startY, endX - startX, endY - startY);

        // Extract the sub-matrix for the ROI
        Mat subMat = new Mat(focusedChannelMat, roi);

        // Calculate the sum of the values in the ROI
        Scalar sumScalar = Core.sumElems(subMat);
        double sum = sumScalar.val[0];

        // Calculate the area of the ROI
        double area = roi.width * roi.height;

        // Calculate and return the density
        return sum / area;
    }

    public static final class RegionOfInterest {
        public double centerX, centerY;
        public final double sizeX, sizeY;

        public RegionOfInterest(double sizeX, double sizeY, double centerX, double centerY) {
            this.sizeX = sizeX;
            this.sizeY = sizeY;

            this.centerX = centerX;
            this.centerY = centerY;
        }
    }
}
