package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * camera profile for april tag camera installed in a fix angle
 */
public final class FixedAngleCameraProfile {
    /**
     * the height of the camera, in reference to the april tags, in cm
     */
    public final double cameraInstallationHeightCM,
    /**
     * the installation angle of the camera, 0 for VERTICALLY DOWNWARDS, positive is to the upper side, in radians
     */
    cameraInstallationAngleRadian,
    /**
     * how much angle of a target, in radian, is altered per unit change in amount of pixels of the object's center in the camera view
     */
    angleRadianPerCameraPixelX,
            angleRadianPerCameraPixelY;

    public final double[] distanceRange;
    public final double[] adjacentTargetRelativeDistanceRange;

    /**
     * create a profile for n april tag camera with fixed angle
     *
     * @param cameraInstallationHeightCM          the height of the camera, in reference to the april tags, in cm
     * @param cameraInstallationAngleRadian       the installation angle of the camera, 0 for VERTICALLY DOWNWARDS, positive is to the upper side, in radians
     * @param angleRadianPerCameraXPixel          how much angle of a target, in radian, is altered per unit change in amount of pixels of the object's center in the camera view
     * @param angleRadianPerCameraYPixel
     * @param distanceRange
     * @param adjacentTargetRelativeDistanceRange
     */
    public FixedAngleCameraProfile(double cameraInstallationHeightCM, double cameraInstallationAngleRadian, double angleRadianPerCameraXPixel, double angleRadianPerCameraYPixel, double[] distanceRange, double[] adjacentTargetRelativeDistanceRange) {
        // TODO add a measuring for x-axis navigation which is quite inaccurate now
        //   use the same method as distance measuring, draw a best-fit-line to measure the fov of the camera, instead of just using the same data as vertical fov
        this.cameraInstallationAngleRadian = cameraInstallationAngleRadian;
        this.cameraInstallationHeightCM = cameraInstallationHeightCM;
        this.angleRadianPerCameraPixelX = angleRadianPerCameraXPixel;
        this.angleRadianPerCameraPixelY = angleRadianPerCameraYPixel;
        this.distanceRange = distanceRange;
        this.adjacentTargetRelativeDistanceRange = adjacentTargetRelativeDistanceRange;
    }

    public double getDistanceFromYPixel(double yPixel) {
        final double targetVerticalAngleRadian = angleRadianPerCameraPixelY * yPixel;
        return cameraInstallationHeightCM * Math.tan(cameraInstallationAngleRadian + targetVerticalAngleRadian);
    }

    /**
     * calculate the angle of the target to the middle line of the camera's view
     * @param pixelX the x position of pixel
     * @return
     */
    public double getTargetAngleRadianFromXPixel(double pixelX) {
        return pixelX * angleRadianPerCameraPixelX + (Math.PI / 2);
    }

    /**
     * measures the profile of the camera
     *
     * @param cameraToTest               the april tag camera to test
     * @param telemetry                  the port to send messages to the control hub
     * @param gamepad                    the game pad to send commands
     * @param cameraInstallationHeightCM the height of the camera installation, need to be measured
     * @param maxDistanceToTarget        the maximum distance to tag for the camera to see the target
     * @param minDistanceToTarget        the minimum distance to tag for the camera to still see the target
     * @param targetID                   the id of the target used in the measurement
     */
    public static void measureCameraVerticalParams(
            RawArilTagRecognitionCamera cameraToTest,
            Telemetry telemetry,
            Gamepad gamepad,
            double cameraInstallationHeightCM,
            double maxDistanceToTarget,
            double minDistanceToTarget,
            int targetID
    ) {
        measureCameraVerticalParams(
                SimpleTargetTrackingCamera.fromAprilTagCamera(cameraToTest, targetID),
                telemetry,
                gamepad,
                cameraInstallationHeightCM,
                maxDistanceToTarget,
                minDistanceToTarget);
    }

    /**
     * measures the profile of the camera
     *
     * @param cameraToTest               the april tag camera to test
     * @param telemetry                  the port to send messages to the control hub
     * @param gamepad                    the game pad to send commands
     * @param cameraInstallationHeightCM the height of the camera installation, need to be measured
     * @param maxDistanceToTarget        the maximum distance to tag for the camera to see the target
     * @param minDistanceToTarget        the minimum distance to tag for the camera to still see the target
     */
    public static void measureCameraVerticalParams(
            RawPixelDetectionCamera cameraToTest,
            Telemetry telemetry,
            Gamepad gamepad,
            double cameraInstallationHeightCM,
            double maxDistanceToTarget,
            double minDistanceToTarget
    ) {
        SimpleTargetTrackingCamera camera = SimpleTargetTrackingCamera.fromPixelCamera(cameraToTest);
        cameraToTest.startRecognizing();

        measureCameraVerticalParams(camera, telemetry, gamepad, cameraInstallationHeightCM, maxDistanceToTarget, minDistanceToTarget);
    }

    public static void measureCameraVerticalParams(SimpleTargetTrackingCamera cameraToTest, Telemetry telemetry, Gamepad gamepad, double cameraInstallationHeightCM, double maxDistanceToTarget, double minDistanceToTarget) {
        final int sampleCount = 4;
        final double[] distanceSamples = new double[sampleCount],  // different distances
                angleSamples = new double[sampleCount],  // the corresponding angle of different distances, calculated with arc tangent
                pixelYSamples = new double[sampleCount]; // the measured pixel y
        for (int i = 0; i < sampleCount; i++) {
            distanceSamples[i] = minDistanceToTarget + i * (maxDistanceToTarget - minDistanceToTarget) / sampleCount;
            angleSamples[i] = Math.atan(distanceSamples[i] / cameraInstallationHeightCM);
        }

        int currentSample = 0;
        for (; ; ) {
            cameraToTest.update();
            telemetry.update();
            telemetry.addData("currently measuring sample", currentSample + 1 + "/" + sampleCount);
            telemetry.addData("please put april tag at distance to camera(cm)", distanceSamples[currentSample]);
            telemetry.addLine("when target in place, press A");
            double[] rawTarget = cameraToTest.getTargetPosition();
            if (rawTarget == null) {
                telemetry.addLine("target unseen");
                continue;
            }

            telemetry.addLine("target seen");
            telemetry.addData("y pixel", rawTarget[1]);
            telemetry.addData("x pixel", rawTarget[0]);
            if (gamepad.a) {
                pixelYSamples[currentSample++] = rawTarget[1];
                try {
                    Thread.sleep(100);
                } catch (InterruptedException ignored) {
                }
            }
            if (currentSample == sampleCount) break;

            telemetry.addLine("press B to terminate");
            if (gamepad.b) return;
        }

        // process statistics
        final double cameraAngleRadianPerPixel = StatisticsUtils.getBestFitLineSlope(pixelYSamples, angleSamples),
                cameraInstallationAngleRadian = StatisticsUtils.getBestFitLineIntersect(pixelYSamples, angleSamples);
        telemetry.update();
        telemetry.addData("camera angle (radian) per pixel", cameraAngleRadianPerPixel);
        telemetry.addData("cameraInstallationAngle (radian)", cameraInstallationAngleRadian);
        telemetry.addData("data correlation squared", Math.pow(StatisticsUtils.getCorrelationCoefficient(pixelYSamples, angleSamples), 2));
        telemetry.update();

        // keep the screen content
        while (!gamepad.b)
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
    }


    /**
     * measures the profile of the camera
     *
     * @param cameraToTest        the april tag camera to test
     * @param telemetry           the port to send messages to the control hub
     * @param gamepad             the game pad to send commands
     * @param distancesToTarget
     * @param maxDistanceToCenter
     * @param targetID            the id of the target used in the measurement
     */
    public static void measureCameraHorizontalParams(
            RawArilTagRecognitionCamera cameraToTest,
            Telemetry telemetry,
            Gamepad gamepad,
            double[] distancesToTarget,
            double maxDistanceToCenter,
            int targetID
    ) {

    }

    public static void measureCameraHorizontalParams(SimpleTargetTrackingCamera cameraToTest, Telemetry telemetry, Gamepad gamepad, double[] distancesToTarget, double maxDistanceToCenter) {
        final double minDistanceToCenter = -maxDistanceToCenter;
        final int sampleCount = 4;
        final double[] samplesCameraAngleRadianPerPixel = new double[distancesToTarget.length];
        for (int distanceCount = 0; distanceCount < distancesToTarget.length; distanceCount++) {
            final double distanceToTarget = distancesToTarget[distanceCount];
            final double[] horizontalDistanceSamples = new double[sampleCount],  // different distances
                    angleSamples = new double[sampleCount],  // the corresponding angle of different distances, calculated with arc tangent
                    pixelXSamples = new double[sampleCount]; // the measured pixel x
            for (int i = 0; i < sampleCount; i++) {
                horizontalDistanceSamples[i] = minDistanceToCenter + i * (maxDistanceToCenter - minDistanceToCenter) / sampleCount;
                angleSamples[i] = Math.atan(horizontalDistanceSamples[i] / distanceToTarget);
            }

            int currentSample = 0;
            for (; ; ) {
                cameraToTest.update();
                telemetry.update();

                telemetry.addData("currently measuring sample", currentSample + 1 + "/" + sampleCount);
                telemetry.addLine("please put the tag at" + distanceToTarget + "cm from the camera");
                telemetry.addData("please put the april tag at a distance to camera view center(cm): ", horizontalDistanceSamples[currentSample]);
                telemetry.addLine("when target in place, press A");
                double[] rawTarget = cameraToTest.getTargetPosition();
                if (rawTarget == null) {
                    telemetry.addLine("target unseen");
                    continue;
                }

                telemetry.addLine("target seen");
                telemetry.addData("y pixel", rawTarget[1]);
                telemetry.addData("x pixel", rawTarget[0]);
                if (gamepad.a) {
                    pixelXSamples[currentSample++] = rawTarget[0];
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException ignored) {
                    }
                }
                if (currentSample == sampleCount) break;

                telemetry.addLine("press B to terminate");
                if (gamepad.b) return;
            }
            // process statistics
            final double sampleCameraAngleRadianPerPixel = StatisticsUtils.getBestFitLineSlope(pixelXSamples, angleSamples);
            samplesCameraAngleRadianPerPixel[distanceCount] = sampleCameraAngleRadianPerPixel;
            telemetry.update();
            telemetry.addData("camera angle radian per pixel(sample" + currentSample + ") is", sampleCameraAngleRadianPerPixel);
            telemetry.addData("data correlation squared", Math.pow(StatisticsUtils.getCorrelationCoefficient(pixelXSamples, angleSamples), 2));
            telemetry.addLine("press X to continue");
            telemetry.update();
            while (!gamepad.x)
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
        }

        telemetry.update();
        telemetry.addData("average camera angle radian per pixel", StatisticsUtils.getMean(samplesCameraAngleRadianPerPixel));
        telemetry.addData("samples standard deviation", StatisticsUtils.getStandardDeviation(samplesCameraAngleRadianPerPixel));
        telemetry.update();

        // keep the screen content
        while (!gamepad.b)
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
    }

    private interface SimpleTargetTrackingCamera {
        /** null for unseen */
        double[] getTargetPosition();
        void update();

        static SimpleTargetTrackingCamera fromAprilTagCamera(RawArilTagRecognitionCamera aprilTagCamera, int targetID) {
            return new SimpleTargetTrackingCamera() {
                @Override
                public double[] getTargetPosition() {
                    RawArilTagRecognitionCamera.AprilTagTargetRaw targetRaw = aprilTagCamera.getRawAprilTagByID(targetID);
                    if (targetRaw == null) return null;
                    return new double[] {targetRaw.x, targetRaw.y};
                }

                @Override
                public void update() {
                    aprilTagCamera.update();
                }
            };
        }

        static SimpleTargetTrackingCamera fromPixelCamera(RawPixelDetectionCamera pixelCamera) {
            return new SimpleTargetTrackingCamera() {
                @Override
                public double[] getTargetPosition() {
                    RawPixelDetectionCamera.PixelTargetRaw targetRaw = pixelCamera.getPixelTargets().isEmpty() ? null : pixelCamera.getPixelTargets().get(0);
                    if (targetRaw == null) return null;
                    return new double[] {targetRaw.x, targetRaw.y};
                }

                @Override
                public void update() { /* do nothing */ }
            };
        }
    }
}
