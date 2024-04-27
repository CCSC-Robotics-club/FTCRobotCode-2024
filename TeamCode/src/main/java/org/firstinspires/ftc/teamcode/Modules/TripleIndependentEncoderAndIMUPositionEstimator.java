package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedIMU;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.AngleUtils;
import org.firstinspires.ftc.teamcode.Utils.PositionEstimator;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.StatisticsUtils;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.SimpleSensor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TripleIndependentEncoderAndIMUPositionEstimator extends RobotModule implements PositionEstimator {
    private final SimpleSensor horizontalEncoder, verticalEncoder1, verticalEncoder2;
    private final ThreadedIMU imu;
    private final double horizontalEncoderFactor, verticalEncoder1Factor, verticalEncoder2Factor, verticalDifferenceToHorizontalBias;
    private Vector2D previousPosition;
    private double horizontalEncoderPreviousReading, verticalEncoder1PreviousReading, verticalEncoder2PreviousReading, imuReading, imuVelocity;

    private Vector2D currentPosition2D;
    /** to the field */
    private Vector2D currentVelocity2D;

    /**
     * the bias between the imu reading to the actual rotation
     * imu + bias = actual
     * */
    private double imuRotationBias;
    private final Map<String, Object> debugMessages = new HashMap<>(1);
    private final boolean positionEncodersAvailable;

    public TripleIndependentEncoderAndIMUPositionEstimator(
            SimpleSensor horizontalEncoder,
            SimpleSensor verticalEncoder1,
            SimpleSensor verticalEncoder2,
            ThreadedIMU imu,
            TripleIndependentEncoderAndIMUSystemParams params
    ) {
        super("position estimator", RobotConfig.ChassisConfigs.positionEstimator_speedEstimationFrequency);
        this.horizontalEncoder = horizontalEncoder;
        this.verticalEncoder1 = verticalEncoder1;
        this.verticalEncoder2 = verticalEncoder2;
        this.imu = imu;

        this.positionEncodersAvailable = params!= null && horizontalEncoder!=null && verticalEncoder1!=null && verticalEncoder2!=null;
        if (positionEncodersAvailable) {
            double encoderValueToCentiMeter = 1.0d / params.encoderValuePerCentiMeter;
            this.horizontalEncoderFactor = (params.horizontalEncoderReversed ? -1 : 1) * encoderValueToCentiMeter;
            this.verticalEncoder1Factor = (params.verticalEncoder1Reversed ? -1 : 1) * encoderValueToCentiMeter;
            this.verticalEncoder2Factor = (params.verticalEncoder2Reversed ? -1 : 1) * encoderValueToCentiMeter;
            this.verticalDifferenceToHorizontalBias = params.horizontalEncoderBiasPerVerticalEncoderDifference;
        } else
            this.horizontalEncoderFactor = this.verticalEncoder1Factor = this.verticalEncoder2Factor = this.verticalDifferenceToHorizontalBias = 0;

        this.currentVelocity2D = new Vector2D();
        this.previousPosition = new Vector2D();
        this.currentPosition2D = new Vector2D();
    }

    @Override
    public void init() {
        reset();
    }
    @Override
    public void periodic(double dt) {
        if (!positionEncodersAvailable) return;
        updateRotation(dt);
        estimatePositions();
        estimateVelocity(dt);
    }

    private void updateRotation(double dt) {
        final long t1 = System.currentTimeMillis();
        final double imuNewReading = imu.getSensorReading();
//        this.imuVelocity = primaryIMU.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        if (dt > 1.0/1000)
            this.imuVelocity = AngleUtils.getActualDifference(imuReading, imuNewReading) / dt; // if dt is too small, we abandon the action since this will make velocity infinity
        this.imuReading = imuNewReading;
        debugMessages.put("imu reading time", System.currentTimeMillis() - t1);
    }

    private void estimatePositions() {
        final double horizontalEncoderReading = horizontalEncoder.getSensorReading() * horizontalEncoderFactor,
                verticalEncoder1Reading =  verticalEncoder1.getSensorReading() * verticalEncoder1Factor,
                verticalEncoder2Reading = verticalEncoder2.getSensorReading() * verticalEncoder2Factor,
                verticalEncoder1Difference = verticalEncoder1Reading - verticalEncoder1PreviousReading,
                verticalEncoder2Difference = verticalEncoder2Reading - verticalEncoder2PreviousReading,
                verticalEncoderMovement = (verticalEncoder1Difference + verticalEncoder2Difference) / 2,
                verticalEncodersDifferentiated = (verticalEncoder1Difference - verticalEncoder2Difference);

        double horizontalEncoderDifference = horizontalEncoderReading - horizontalEncoderPreviousReading;

        debugMessages.put("horizontal enc val", horizontalEncoderDifference);
        debugMessages.put("vertical enc cor", verticalEncodersDifferentiated * verticalDifferenceToHorizontalBias);
        horizontalEncoderDifference -= verticalEncodersDifferentiated * verticalDifferenceToHorizontalBias;

        Vector2D translationalDifference = new Vector2D(new double[] {
                horizontalEncoderDifference,
                verticalEncoderMovement
        });

        this.currentPosition2D = currentPosition2D.addBy(translationalDifference.multiplyBy(
                new Rotation2D(getRotation())
        ));

        horizontalEncoderPreviousReading = horizontalEncoderReading;
        verticalEncoder1PreviousReading = verticalEncoder1Reading;
        verticalEncoder2PreviousReading = verticalEncoder2Reading;
    }

    private void estimateVelocity(double dt) {
        if (dt < 1e-4) return;
        Vector2D positionDifference = currentPosition2D.addBy(previousPosition.multiplyBy(-1));
        this.currentVelocity2D = positionDifference.multiplyBy(1/dt);

        previousPosition = currentPosition2D;
    }

    @Override
    protected void onDestroy() {

    }

    @Override
    public void reset() {
        this.horizontalEncoderPreviousReading = horizontalEncoder.getSensorReading() * horizontalEncoderFactor;
        this.verticalEncoder1PreviousReading = verticalEncoder1.getSensorReading() * verticalEncoder1Factor;
        this.verticalEncoder2PreviousReading = verticalEncoder2.getSensorReading() * verticalEncoder2Factor;
        this.imuRotationBias = 0;

        calibratePosition();
        calibrateRotation();

        this.imuReading = 0;
        this.imuVelocity = 0;
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        debugMessages.put("vel (robot)", getCurrentVelocity(Chassis.OrientationMode.ROBOT_ORIENTATED));
        // debugMessages.put("vel (field)", getCurrentVelocity(ChassisModule.OrientationMode.FIELD_ORIENTATED));
        debugMessages.put("position", getCurrentPosition());
        debugMessages.put("robot yaw", getRotation());
        debugMessages.put("robot yaw velocity", getAngularVelocity());
        debugMessages.put("update rate", getUpdateCountPerSecond());
        return debugMessages;
    }

    @Override
    public void setCurrentPosition(Vector2D currentPosition) {
        this.currentPosition2D = currentPosition;
    }

    @Override
    public Vector2D getCurrentVelocity(Chassis.OrientationMode orientationMode) {
        switch (orientationMode) {
            case FIELD_ORIENTATED:
                return currentVelocity2D;
            case ROBOT_ORIENTATED:
                return getCurrentVelocity(Chassis.OrientationMode.FIELD_ORIENTATED).multiplyBy(
                        new Rotation2D(getRotation()).getReversal()
                );
            default:
                throw new IllegalArgumentException("unknown orientation mode" + orientationMode.name());
        }
    }

    @Override
    public double getRotation() {
        return AngleUtils.simplifyAngle(imuReading + imuRotationBias);
    }

    @Override
    public double getAngularVelocity() {
        return imuVelocity;
    }

    /**
     * set a given rotation to be the current reading
     * @param givenRotation the rotation given in radian
     * */
    @Override
    public void setRotation(double givenRotation) {
        resetIMU();
        this.imuRotationBias = givenRotation;
    }

    private void resetIMU() {
        this.imu.resetYaw();
    }

    @Override
    public Vector2D getCurrentPosition() {
        return currentPosition2D;
    }

    @Override
    public void forceUpdate() {
        super.periodic();
    }

    public static final class TripleIndependentEncoderAndIMUSystemParams {
        public final boolean horizontalEncoderReversed, verticalEncoder1Reversed, verticalEncoder2Reversed;
        public final double encoderValuePerCentiMeter, horizontalEncoderBiasPerVerticalEncoderDifference;
        public TripleIndependentEncoderAndIMUSystemParams(boolean horizontalEncoderReversed, boolean verticalEncoder1Reversed, boolean verticalEncoder2Reversed, double encoderValuePerCentiMeter, double horizontalEncoderBiasPerVerticalEncoderDifference) {
            this.horizontalEncoderReversed = horizontalEncoderReversed;
            this.verticalEncoder1Reversed = verticalEncoder1Reversed;
            this.verticalEncoder2Reversed = verticalEncoder2Reversed;

            this.encoderValuePerCentiMeter = encoderValuePerCentiMeter;
            this.horizontalEncoderBiasPerVerticalEncoderDifference = horizontalEncoderBiasPerVerticalEncoderDifference;
        }

        public static void encoderToYawMeasuring(
                DcMotorEx horizontalEncoder,
                DcMotorEx verticalEncoder1,
                DcMotorEx verticalEncoder2,
                boolean[] encoderReversed,
                DcMotor frontLeftMotor,
                DcMotor frontRightMotor,
                DcMotor backLeftMotor,
                DcMotor backRightMotor,
                IMU imu,
                Gamepad testGamePad,
                Telemetry telemetry
        ) {
            final int loops = 10;

            double previousRotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), radianRotated = 0;

            int[] encoderStartingReadings = new int[] {horizontalEncoder.getCurrentPosition(), verticalEncoder1.getCurrentPosition(), verticalEncoder2.getCurrentPosition()};

            for(;;) {
                if (testGamePad.a) {
                    frontLeftMotor.setPower(-0.5);
                    backLeftMotor.setPower(-0.5);
                    frontRightMotor.setPower(-0.5);
                    backRightMotor.setPower(-0.5);
                } else {
                    frontLeftMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backRightMotor.setPower(0);
                }

                int verticalEncoder1Reading = (verticalEncoder1.getCurrentPosition() - encoderStartingReadings[1]) * (encoderReversed[1] ? -1:1),
                        verticalEncoder2Reading = (verticalEncoder2.getCurrentPosition() - encoderStartingReadings[2]) * (encoderReversed[2] ? -1:1),
                        horizontalEncoderReading = (horizontalEncoder.getCurrentPosition() - encoderStartingReadings[0]) * (encoderReversed[0] ? -1:1);

                double currentRotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double rotationDifference = AngleUtils.getActualDifference(previousRotation, currentRotation);
                radianRotated += rotationDifference;
                if (radianRotated / (Math.PI * 2) >=  loops) {
                    int verticalDifferencePerResolution = (int)
                            ((verticalEncoder2Reading - verticalEncoder1Reading)
                                    / (radianRotated / (Math.PI * 2)));
                    int horizontalDifferencePerResolution = (int)
                            ((verticalEncoder2Reading - horizontalEncoderReading)
                                    / (radianRotated / (Math.PI * 2)));;
                    telemetry.addData("verticalDifferencePerResolution", verticalDifferencePerResolution); // -19855
                    telemetry.addData("horizontalDifferencePerResolution", horizontalDifferencePerResolution); // -6658
                    break;
                }

                telemetry.addLine("testing... press A again to cancel");
                telemetry.addData("angle rotated", radianRotated);
                telemetry.addData("vertical difference", verticalEncoder2Reading - verticalEncoder1Reading);
                telemetry.addData("horizontal bias", horizontalEncoderReading);
                telemetry.update();

                previousRotation = currentRotation;
            }

            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);

            while (true) telemetry.update();
        }
    }

    public static void verticalDifferencesToHorizontalBiasMeasuring(
            DcMotorEx horizontalEncoder,
            DcMotorEx verticalEncoder1,
            DcMotorEx verticalEncoder2,
            boolean[] encoderReversed,
            DcMotor frontLeftMotor,
            DcMotor frontRightMotor,
            DcMotor backLeftMotor,
            DcMotor backRightMotor,
            Gamepad testGamePad,
            Telemetry telemetry,
            SequentialCommandSegment.IsCompleteChecker isCompleteChecker
    ) {
        List<Double> horizontalBias = new ArrayList<>(1);
        List<Double> verticalEncoderDifferences = new ArrayList<>(1);

        int horizontalEncoderPreviousReading = horizontalEncoder.getCurrentPosition(), verticalEncoder1PreviousReading = verticalEncoder1.getCurrentPosition(), verticalEncoder2PreviousReading = verticalEncoder2.getCurrentPosition();
        long previousTimeMillis = System.currentTimeMillis();
        while(!isCompleteChecker.isComplete()) {
            double motorPower = 0;
            if (testGamePad.a)
                motorPower = 0.25;
            else if (testGamePad.b)
                motorPower = 0.5;
            else if (testGamePad.y)
                motorPower = 0.75;

            frontLeftMotor.setPower(motorPower);
            backLeftMotor.setPower(motorPower);
            frontRightMotor.setPower(motorPower);
            backRightMotor.setPower(motorPower);

            if (System.currentTimeMillis() - previousTimeMillis > 2000) {
                int horizontalEncoderValue = horizontalEncoder.getCurrentPosition() - horizontalEncoderPreviousReading,
                        verticalEncoder1Value = verticalEncoder1.getCurrentPosition() - verticalEncoder1PreviousReading,
                        verticalEncoder2Value = verticalEncoder2.getCurrentPosition() - verticalEncoder2PreviousReading;
                telemetry.addData("enc 1 raw", verticalEncoder1.getCurrentPosition());
                telemetry.addData("hor enc val", horizontalEncoderValue * (encoderReversed[0] ? -1:1));
                telemetry.addData("ver enc (1) val", verticalEncoder1Value * (encoderReversed[1] ? -1:1));
                telemetry.addData("ver enc (2) val", verticalEncoder2Value * (encoderReversed[2] ? -1:1));

                if (Math.abs(verticalEncoder2Value) > 100 && Math.abs(verticalEncoder1Value) > 100 && Math.abs(horizontalEncoderValue) > 100){
                    double verticalDifferenceSpeed = verticalEncoder1Value * (encoderReversed[1] ? -1:1) - verticalEncoder2Value * (encoderReversed[2] ? -1:1);
                    telemetry.addData("vertical speed", verticalDifferenceSpeed);
                    double horizontalSpeed = horizontalEncoderValue * (encoderReversed[0] ? -1:1);
                    horizontalBias.add(horizontalSpeed);
                    verticalEncoderDifferences.add(verticalDifferenceSpeed);
                    telemetry.addData("horizontal speed", horizontalSpeed);
                    telemetry.addData("speed proportion", horizontalSpeed / verticalDifferenceSpeed);
                    horizontalEncoderPreviousReading += horizontalEncoderValue;
                    verticalEncoder1PreviousReading += verticalEncoder1Value;
                    verticalEncoder2PreviousReading += verticalEncoder2Value;
                    previousTimeMillis = System.currentTimeMillis();
                }
                telemetry.addLine("press X to see results");
                telemetry.update();
            }
            if (testGamePad.x)
                break;
        }

        final double coe = StatisticsUtils.getCorrelationCoefficient(StatisticsUtils.toArray(verticalEncoderDifferences), StatisticsUtils.toArray(horizontalBias)),
                slope = StatisticsUtils.getBestFitLineSlope(StatisticsUtils.toArray(verticalEncoderDifferences), StatisticsUtils.toArray(horizontalBias)),
                inters = StatisticsUtils.getBestFitLineIntersect(StatisticsUtils.toArray(verticalEncoderDifferences), StatisticsUtils.toArray(horizontalBias));
        telemetry.update(); // clear screen
        telemetry.addData("r square", coe * coe);
        telemetry.addData("horizontal bias / vertical diff", slope);
        telemetry.addData("result error", inters);

        telemetry.addLine("press A to end");
        telemetry.update();
        while (!testGamePad.a)
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
    }
}
