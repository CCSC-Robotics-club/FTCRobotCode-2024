package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils.AngleUtils;
import org.firstinspires.ftc.teamcode.Utils.PositionEstimator;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.Transformation2D;
import org.firstinspires.ftc.teamcode.Utils.Vector2D;

import java.util.HashMap;
import java.util.Map;

public class IndependentEncoderAndIMUPositionEstimator extends RobotModule implements PositionEstimator { // TODO add owner judgement when finish testing
    private final DcMotor quadratureDecodingEncoderHorizontal, quadratureDecodingEncoderVertical;
    private final IMU imu;
    private final double horizontalEncoderBiasPerRadian, verticalEncoderBiasPerRadian;
    private final Transformation2D encoderToActualSpaceTransform;
    private Vector2D currentPosition, currentVelocityToSelf;
    private double horizontalEncoderPreviousReading, verticalEncoderPreviousReading, previousRotation;

    // debug messages
    private double doubleToPrint = 0;
    private double previousDT = 0;

    /**
     * creates a position estimator based on independent encoders
     * @param quadratureDecodingEncoderHorizontal the encoder that is installed pointing to the horizontal direction
     * @param quadratureDecodingEncoderVertical the encoder that is installed pointing to the vertical direction
     * @param imu the imu of the robot
     * @param horizontalEncoderBiasPerIMURadian the amount of bias the horizontal encoder will read when rotating the robot ONE LOOP COUNTERCLOCKWISE without moving the robot translationally (-9934)
     * @param verticalEncoderBiasPerIMURadian the amount of bias the horizontal encoder will read when rotating the robot ONE LOOP COUNTERCLOCKWISE without moving the robot translationally (2534)
     * @param horizontalEncoderPointing the exact pointing direction of the horizontal encoder, in reference to the robot. The encoder should receive POSITIVE readings when travelling to this direction
     * @param verticalEncoderPointing the exact pointing direction of the vertical encoder, in reference to the robot. The encoder should receive POSITIVE readings when travelling to this direction
     */
    public IndependentEncoderAndIMUPositionEstimator(DcMotor quadratureDecodingEncoderHorizontal, DcMotor quadratureDecodingEncoderVertical, IMU imu, double horizontalEncoderBiasPerIMURadian, double verticalEncoderBiasPerIMURadian, Vector2D horizontalEncoderPointing, Vector2D verticalEncoderPointing) {
        super("encoder-based position estimator");
        this.quadratureDecodingEncoderHorizontal = quadratureDecodingEncoderHorizontal;
        this.quadratureDecodingEncoderVertical = quadratureDecodingEncoderVertical;
        this.imu = imu;
        this.encoderToActualSpaceTransform = new Transformation2D(horizontalEncoderPointing, verticalEncoderPointing);
        this.horizontalEncoderBiasPerRadian = horizontalEncoderBiasPerIMURadian;
        this.verticalEncoderBiasPerRadian = verticalEncoderBiasPerIMURadian;
    }

    /**
     * creates a position estimator based on independent encoders
     * @param quadratureDecodingEncoderHorizontal the encoder that is installed pointing to the horizontal direction
     * @param quadratureDecodingEncoderVertical the encoder that is installed pointing to the vertical direction
     * @param imu the imu of the robot
     * @param horizontalEncoderBiasPerRobotRevolution the amount of bias the horizontal encoder will read when rotating the robot ONE LOOP COUNTERCLOCKWISE without moving the robot translationally
     * @param verticalEncoderBiasPerRobotRevolution the amount of bias the horizontal encoder will read when rotating the robot ONE LOOP COUNTERCLOCKWISE without moving the robot translationally
     * */
    public IndependentEncoderAndIMUPositionEstimator(DcMotor quadratureDecodingEncoderHorizontal, DcMotor quadratureDecodingEncoderVertical, IMU imu, double horizontalEncoderBiasPerRobotRevolution, double verticalEncoderBiasPerRobotRevolution) {
        this(quadratureDecodingEncoderHorizontal, quadratureDecodingEncoderVertical, imu, horizontalEncoderBiasPerRobotRevolution, verticalEncoderBiasPerRobotRevolution,
                Transformation2D.originalSpace.getIHat(),
                Transformation2D.originalSpace.getJHat()
        );
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    public void reset() {
        calibratePosition();

        this.horizontalEncoderPreviousReading = quadratureDecodingEncoderHorizontal.getCurrentPosition();
        this.verticalEncoderPreviousReading = quadratureDecodingEncoderVertical.getCurrentPosition();
        this.previousRotation = getRotation();
        this.currentVelocityToSelf = new Vector2D();
    }

    @Override
    public void periodic(double dt) {
        if (dt < 10e-7) return; // avoid zero division

        Vector2D instantDisplacement = getInstanceDisplacement();
        this.currentVelocityToSelf = instantDisplacement.multiplyBy(1/dt);
        this.currentPosition = currentPosition.addBy(
                getCurrentVelocity(ChassisModule.OrientationMode.FIELD_ORIENTATED).multiplyBy(dt)
        );

        horizontalEncoderPreviousReading = quadratureDecodingEncoderHorizontal.getCurrentPosition();
        verticalEncoderPreviousReading = quadratureDecodingEncoderVertical.getCurrentPosition();

        this.previousDT = dt;
    }


    private Vector2D getInstanceDisplacement() {
        double horizontalEncoderDifference = quadratureDecodingEncoderHorizontal.getCurrentPosition() - horizontalEncoderPreviousReading,
            verticalEncoderDifference = quadratureDecodingEncoderVertical.getCurrentPosition() - verticalEncoderPreviousReading;

        double dRotation = AngleUtils.getActualDifference(previousRotation, getRotation());
        horizontalEncoderDifference -= dRotation * horizontalEncoderBiasPerRadian;
        verticalEncoderDifference -= dRotation * verticalEncoderBiasPerRadian;

        previousRotation = getRotation();

        this.doubleToPrint = horizontalEncoderDifference;
        return new Vector2D(new double[] {horizontalEncoderDifference, verticalEncoderDifference}).multiplyBy(encoderToActualSpaceTransform); // TODO this does not seem to be working
    }


    @Override
    public void setCurrentPosition(Vector2D currentPosition) {
        this.currentPosition = currentPosition;
    }

    @Override
    public Vector2D getCurrentVelocity(ChassisModule.OrientationMode orientationMode) {
        switch (orientationMode) {
            case FIELD_ORIENTATED:
                return getCurrentVelocity(ChassisModule.OrientationMode.ROBOT_ORIENTATED).multiplyBy(
                        new Rotation2D(getRotation())
                );
            case ROBOT_ORIENTATED:
                return this.currentVelocityToSelf;
            default:
                throw new IllegalArgumentException("unsupported orientation mode" + orientationMode.name());
        }
    }

    @Override
    public double getRotation() {
        return AngleUtils.simplifyAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    @Override
    public double getAngularVelocity() {
        return imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    @Override
    public void calibrateRotation() {
        this.imu.resetYaw();
    }

    @Override
    public void setRotation(double givenRotation) {

    }

    @Override
    public Vector2D getCurrentPosition() {
        return currentPosition;
    }

    @Override
    public void forceUpdate() {

    }

    @Override
    public void onDestroy() {

    }

    @Override
    public Map<String, Object> getDebugMessages() {
       Map<String, Object> messages = new HashMap<>();
       messages.put("horizontal encoder change", doubleToPrint);

       messages.put("dt(ms)", (int)(1000*previousDT));
       messages.put("pos", getCurrentPosition());
       messages.put("vel", getCurrentVelocity(ChassisModule.OrientationMode.ROBOT_ORIENTATED));
       return messages;
    }

    public static void measureParams(
            DcMotor encoder,
            IMU imu,
            DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor,
            Gamepad testGamePad,
            Telemetry telemetry
    ) {
        int startingEncoderValue = encoder.getCurrentPosition(), previousEncoderValue = startingEncoderValue;
        double imuTurned = 0, imuPreviousReading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        for(;;) {
            double dIMU = AngleUtils.getActualDifference(imuPreviousReading, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            int dEncoder = encoder.getCurrentPosition() - previousEncoderValue;

            imuTurned += dIMU;
            int encoderTurned = encoder.getCurrentPosition() - startingEncoderValue;
            telemetry.addLine("sum up:");
            telemetry.addData("IMU total", imuTurned);
            telemetry.addData("encoder Reading", encoderTurned);
            if (Math.abs(encoderTurned) > 100)
                telemetry.addData("proportion", encoderTurned / imuTurned);
            telemetry.addData("encoder Raw", encoder.getCurrentPosition());

            telemetry.addLine();
            telemetry.addLine("instance");
            telemetry.addData("dIMU", dIMU);
            telemetry.addData("d encoder", dEncoder);
            if (Math.abs(dEncoder) > 10)
                telemetry.addData("proportion", dEncoder / dIMU);
            telemetry.update();

            if (testGamePad.y) {
                frontLeftMotor.setPower(0.7);
                backLeftMotor.setPower(0.7);
                frontRightMotor.setPower(0.7);
                backRightMotor.setPower(0.7);
            } else if (testGamePad.a) {
                frontLeftMotor.setPower(0.3);
                backLeftMotor.setPower(0.3);
                frontRightMotor.setPower(0.3);
                backRightMotor.setPower(0.3);
            } else {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            }

            previousEncoderValue = encoder.getCurrentPosition();
            imuPreviousReading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            try {Thread.sleep(80); }
            catch (InterruptedException ignored) {}

            if (testGamePad.b) break;
        }
    }
}
