package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMUNew;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.EncoderMotorWheel;
import org.firstinspires.ftc.teamcode.Modules.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Modules.FixedAnglePixelCamera;
import org.firstinspires.ftc.teamcode.Modules.TripleIndependentEncoderAndIMUPositionEstimator;
import org.firstinspires.ftc.teamcode.Services.AutoProgramRunner;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.Claw;
import org.firstinspires.ftc.teamcode.Utils.DualServoClaw;
import org.firstinspires.ftc.teamcode.Utils.FixedAngleCameraProfile;
import org.firstinspires.ftc.teamcode.Utils.HuskyAprilTagCamera;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.PixelCameraAimBot;
import org.firstinspires.ftc.teamcode.Utils.RawPixelDetectionCamera;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.SimpleFeedForwardSpeedController;
import org.firstinspires.ftc.teamcode.Utils.SingleServoClaw;
import org.firstinspires.ftc.teamcode.Utils.TensorCamera;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

@TeleOp(name="Test_main")
public class TestMain extends LinearOpMode {
    @Override
    public void runOpMode() {
        motorsMatch();
    }

    List<RobotModule> robotModules = new ArrayList<>(1);
    TelemetrySender telemetrySender;
    private Chassis getChassisModuleWithDefaultConfig() {
//        FixedAngleArilTagCamera camera = getHuskyWithDefaultConfig();

        /* hardware */
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(RobotConfig.testConfig.imuParameter);
        SimpleFeedForwardSpeedController.SpeedControllerProfile wheelSpeedControllerProfile =
                new SimpleFeedForwardSpeedController.SpeedControllerProfile(
                        RobotConfig.ChassisConfigs.wheel_proportionGain, RobotConfig.ChassisConfigs.wheel_feedForwardGain, RobotConfig.ChassisConfigs.wheel_feedForwardDelay);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        EncoderMotorWheel frontLeftWheel = new EncoderMotorWheel(
                frontLeftMotor,
                frontLeftMotor,
                new SimpleFeedForwardSpeedController(wheelSpeedControllerProfile),
                RobotConfig.ChassisConfigs.wheel_maxVelocity
        );
        EncoderMotorWheel frontRightWheel = new EncoderMotorWheel(
                frontRightMotor,
                frontRightMotor,
                new SimpleFeedForwardSpeedController(wheelSpeedControllerProfile),
                RobotConfig.ChassisConfigs.wheel_maxVelocity
        );
        EncoderMotorWheel backLeftWheel = new EncoderMotorWheel(
                backLeftMotor,
                backLeftMotor,
                new SimpleFeedForwardSpeedController(wheelSpeedControllerProfile),
                RobotConfig.ChassisConfigs.wheel_maxVelocity
        );
        EncoderMotorWheel backRightWheel = new EncoderMotorWheel(
                backRightMotor,
                backRightMotor,
                new SimpleFeedForwardSpeedController(wheelSpeedControllerProfile),
                RobotConfig.ChassisConfigs.wheel_maxVelocity
        );
        frontLeftWheel.setEncoderReversed(RobotConfig.testConfig.frontLeftWheel_encoderReversed);
        frontRightWheel.setEncoderReversed(RobotConfig.testConfig.frontRightWheel_encoderReversed);
        backLeftWheel.setEncoderReversed(RobotConfig.testConfig.backLeftWheel_encoderReversed);
        backRightWheel.setEncoderReversed(RobotConfig.testConfig.backRightWheel_encoderReversed);

        frontLeftWheel.setMotorReversed(RobotConfig.testConfig.frontLeftWheel_motorReversed);
        frontRightWheel.setMotorReversed(RobotConfig.testConfig.frontRightWheel_motorReversed);
        backLeftWheel.setMotorReversed(RobotConfig.testConfig.backLeftWheel_motorReversed);
        backRightWheel.setMotorReversed(RobotConfig.testConfig.backRightWheel_motorReversed);

        final RobotConfig.HardwareConfigs hardwareConfigs = RobotConfig.hardwareConfigs_2024Competition;
        TripleIndependentEncoderAndIMUPositionEstimator positionEstimator = new TripleIndependentEncoderAndIMUPositionEstimator(
                hardwareMap.get(DcMotor.class, hardwareConfigs.encoderNames[0]),
                hardwareMap.get(DcMotor.class, hardwareConfigs.encoderNames[1]),
                hardwareMap.get(DcMotor.class, hardwareConfigs.encoderNames[2]),
                imu,
                hardwareConfigs.encodersParams
        );
        positionEstimator.init();
        robotModules.add(positionEstimator);

        Chassis chassis = new Chassis(frontLeftWheel, frontRightWheel, backLeftWheel ,backRightWheel, positionEstimator, null, FixedAngleArilTagCamera.WallTarget.Name.RED_ALLIANCE_WALL);

        // camera.init();
        frontRightWheel.init();
        frontLeftWheel.init();
        backRightWheel.init();
        backLeftWheel.init();
        chassis.init();


        // robotModules.add(camera);
        robotModules.add(frontLeftWheel);
        robotModules.add(frontRightWheel);
        robotModules.add(backLeftWheel);
        robotModules.add(backRightWheel);
        robotModules.add(chassis);

        this.telemetrySender = new TelemetrySender(telemetry);
        telemetrySender.init();

        telemetrySender.addRobotModule(chassis);
        telemetrySender.addRobotModule(positionEstimator);
//        telemetrySender.addRobotModule(camera);

        chassis.gainOwnerShip(null);
        return chassis;
    }

    private FixedAngleArilTagCamera getHuskyWithDefaultConfig() {
        return new FixedAngleArilTagCamera(new HuskyAprilTagCamera(hardwareMap.get(HuskyLens.class, "husky")), RobotConfig.VisualNavigationConfigs.visualCameraProfile);
    }

    private FixedAnglePixelCamera getPixelCameraWithDefaultConfig() {
        TensorCamera tensorCamera = new TensorCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        return new FixedAnglePixelCamera(
                tensorCamera,
                RobotConfig.VisualNavigationConfigs.pixelCameraSetUpProfile,
                RobotConfig.VisualNavigationConfigs.pixelCameraInstallFacing
        );
    }

    private void updateRobot() {
        for (RobotModule robotModule : robotModules)
            robotModule.periodic();
        telemetrySender.periodic();
    }
    public void motorsMatch() {
        DcMotorEx mot0 = hardwareMap.get(DcMotorEx.class, "mot0");
        DcMotorEx mot1 = hardwareMap.get(DcMotorEx.class, "mot1");
        DcMotorEx mot2 = hardwareMap.get(DcMotorEx.class, "mot2");
        DcMotorEx mot3 = hardwareMap.get(DcMotorEx.class, "mot3");

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("mot0 enc", mot0.getVelocity());
            telemetry.addData("mot1 enc", mot1.getVelocity());
            telemetry.addData("mot2 enc", mot2.getVelocity());
            telemetry.addData("mot3 enc", mot3.getVelocity());
            telemetry.update();
            mot0.setPower(gamepad1.a ? 0.5:0);
            mot1.setPower(gamepad1.b ? 0.5:0);
            mot2.setPower(gamepad1.x ? 0.5:0);
            mot3.setPower(gamepad1.y ? 0.5:0);
        }
    }

    public void fourWheelSpeedTest() {
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            frontLeft.setPower(gamepad1.a ? 0.5:0);
            frontRight.setPower(gamepad1.b ? 0.5:0);
            backLeft.setPower(gamepad1.x ? 0.5:0);
            backRight.setPower(gamepad1.y ? 0.5:0);

            telemetry.addData("front left motor speed", frontLeft.getVelocity());
            telemetry.addData("front right motor speed", frontRight.getVelocity());
            telemetry.addData("back left motor speed", backLeft.getVelocity());
            telemetry.addData("back right motor speed", backRight.getVelocity());

            telemetry.update();
        }
    }

    private void intakeAndArmTest() {
        final DcMotor intakeMotor1 = hardwareMap.get(DcMotor.class, "intake1"),
                intakeMotor2 = hardwareMap.get(DcMotor.class, "intake2"),
                armMotor = hardwareMap.get(DcMotor.class, "arm");
        intakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        double power = 0;
        while (!isStopRequested() && opModeIsActive()) {
            if (Math.abs(gamepad1.left_stick_y) > 0.05)
                power += gamepad1.left_stick_y * 0.05 * -1.5;
            if (gamepad1.a)
                intakeMotor1.setPower(-power);
            else
                intakeMotor1.setPower(0);

            if (gamepad1.b)
                intakeMotor2.setPower(power);
            else
                intakeMotor2.setPower(0);

            if (gamepad1.x)
                armMotor.setPower(power);
            else
                armMotor.setPower(0);

            if (power > 1) power = 1;
            else if (power < -1) power = -1;
            telemetry.addData("mot pow", power);
            telemetry.addData("controller input", gamepad1.left_stick_y);
            telemetry.update();

            sleep(50);
        }
    }

    public void singleWheelPIDTest() {
        SimpleFeedForwardSpeedController.SpeedControllerProfile wheelSpeedControllerProfile =
                new SimpleFeedForwardSpeedController.SpeedControllerProfile(
                        RobotConfig.ChassisConfigs.wheel_proportionGain, RobotConfig.ChassisConfigs.wheel_feedForwardGain, RobotConfig.ChassisConfigs.wheel_feedForwardDelay);
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        EncoderMotorWheel testWheel = new EncoderMotorWheel(
                motor,
                motor,
                new SimpleFeedForwardSpeedController(wheelSpeedControllerProfile),
                RobotConfig.ChassisConfigs.wheel_maxVelocity
        );

        testWheel.reset();
        testWheel.setEncoderReversed(true);
        testWheel.setEncoderEnabled(true);

        waitForStart();
        long previousTime = System.currentTimeMillis();

        while (!isStopRequested() && opModeIsActive()) {
            if (Math.abs(gamepad1.right_stick_y) > 0.05) {
                testWheel.setEnabled(true);
                testWheel.setVelocity(gamepad1.right_stick_y * -1);
            } else
                testWheel.setEnabled(false);

            testWheel.periodic((System.currentTimeMillis() - previousTime) / 1000.0f);

            previousTime = System.currentTimeMillis();
        }
    }

    private void servoTest() {
        Servo servo1 = hardwareMap.get(Servo.class, "launch");
        Servo servo2 = hardwareMap.get(Servo.class, "extend");

        waitForStart();

        double servoAngle = 0;
        long previousTime = System.currentTimeMillis();

        while (!isStopRequested() && opModeIsActive()) {
            double dt = (System.currentTimeMillis() - previousTime) / 1000.0f;
            if (Math.abs(gamepad1.left_stick_y) > 0.05)
                servoAngle += dt * gamepad1.left_stick_y * -5;
            if (gamepad1.a)
                servoAngle = 0;
            servoAngle = Math.min(Math.max(0,servoAngle), 1);
            servo1.setPosition(servoAngle);
            servo2.setPosition(servoAngle);
            telemetry.addData("servo angle", servoAngle);
            telemetry.update();
            previousTime = System.currentTimeMillis();
        }
    }

    private Claw getSingleServoClaw() {
        final Servo servo = hardwareMap.get(Servo.class, "claw1");

        return new SingleServoClaw(servo, new Claw.ServoProfile(1, 0.8));
    }

    private Claw getDualServoClaw() {
        final Servo servo1 = hardwareMap.get(Servo.class, "claw1"),
                servo2 = hardwareMap.get(Servo.class, "claw2");
        return new DualServoClaw(servo1, servo2, new Claw.ServoProfile(1-0.4, 1-0.16), new Claw.ServoProfile(0.4, 0.16));
    }

    private void armAndClawTest() {
        final DcMotor armMotor = hardwareMap.get(DcMotor.class, "arm");
        final Claw claw = getSingleServoClaw();

        waitForStart();

        final double powerRate = 0.75;
        final int startingPos = armMotor.getCurrentPosition();

        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.a)
                claw.close();
            else if (gamepad1.b)
                claw.open();

            double power = gamepad1.left_stick_y * powerRate;
            if (Math.abs(power) < 0.05)
                power = 0;

            armMotor.setPower(power);

            telemetry.addData("claw is", claw.isClosed() ? "closed" : "open");
            telemetry.addData("arm encoder reading", armMotor.getCurrentPosition() - startingPos);
            telemetry.update();

            sleep(50);
        }
    }
    private void dualServoClawAndArmTest() {
        final Servo servo1 = hardwareMap.get(Servo.class, "claw1"),
                servo2 = hardwareMap.get(Servo.class, "claw2");
        final DcMotor armMotor = hardwareMap.get(DcMotor.class, "arm");
        Claw claw = new DualServoClaw(servo1, servo2, new Claw.ServoProfile(1-0.6, 1-0.85), new Claw.ServoProfile(0.6, 0.85));

        waitForStart();

        final double powerRate = 0.75;
        final int startingPos = armMotor.getCurrentPosition();

        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.a)
                claw.close();
            else if (gamepad1.b)
                claw.open();

            double power = gamepad1.left_stick_y * powerRate;
            if (Math.abs(power) < 0.05)
                power = 0;

            armMotor.setPower(power);

            telemetry.addData("claw is", claw.isClosed() ? "closed" : "open");
            telemetry.addData("arm encoder reading", armMotor.getCurrentPosition() - startingPos);
            telemetry.update();

            sleep(50);
        }
    }

    private void servoTriggerTest() {
        Servo trigger = hardwareMap.get(Servo.class, "trigger");

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.b) {
                trigger.setPosition(0.85);
            } else
                trigger.setPosition(0.55);
        }
    }

    private void concept360Servo() {
        Servo servo360 = hardwareMap.get(Servo.class, "servo360");

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            double power = -gamepad1.left_stick_y;
            if (Math.abs(power) < 0.05) power = 0;

            servo360.setPosition(power*0.5 + 0.5);

            telemetry.addData("pow", power);
            telemetry.update();
            sleep(50);
        }
    }

    private void armTest() {
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "arm");

        waitForStart();

        final double powerRate = 0.75;
        final int startingPos = armMotor.getCurrentPosition();
        while (!isStopRequested() && opModeIsActive()) {
            double power = gamepad1.left_stick_y * powerRate;
            if (Math.abs(power) < 0.05)
                power = 0;

            armMotor.setPower(power);

            telemetry.addData("arm encoder reading", armMotor.getCurrentPosition() - startingPos);
            telemetry.update();
            sleep(50);
        }
    }

    private void singleEncoderTest() {
        DcMotor testEncoder = hardwareMap.get(DcMotor.class, "backRight");

        waitForStart();

        double calibratedValue = 0;
        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("test encoder reading", testEncoder.getCurrentPosition() - calibratedValue);

            telemetry.update();

            if (gamepad1.a)
                calibratedValue = testEncoder.getCurrentPosition();
        }
    }

    private void imuTest() {
        AdafruitBNO055IMUNew imu = hardwareMap.get(AdafruitBNO055IMUNew.class, "testIMU");
        imu.initialize();

        waitForStart();
        imu.resetYaw();

        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.a)
                imu.resetYaw();
            telemetry.addData("imu yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.update();
        }
    }

    /**
     * measurement for EncoderBiasPerRadian
     */
    private void encoderParamsMeasuring() {
        DcMotorEx verticalEncoder1 = hardwareMap.get(DcMotorEx.class, "frontLeft"); // vertical 1
        DcMotorEx verticalEncoder2 = hardwareMap.get(DcMotorEx.class, "frontRight"); // vertical 2
        DcMotorEx horizontalEncoder = hardwareMap.get(DcMotorEx.class, "backRight"); // horizontal

        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft"),
                frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight"),
                backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft"),
                backRightMotor = hardwareMap.get(DcMotor.class, "backRight");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(RobotConfig.testConfig.imuParameter);

        waitForStart();

        TripleIndependentEncoderAndIMUPositionEstimator.verticalDifferencesToHorizontalBiasMeasuring(
                horizontalEncoder,
                verticalEncoder1,
                verticalEncoder2,
                new boolean[] {true, true, false},
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                gamepad1,
                telemetry
        );
    }

    public void huskyTest() {
        HuskyLens camera = hardwareMap.get(HuskyLens.class, "husky");
        camera.initialize();
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.a) camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION); // TODO use this as detection
            if (gamepad1.b) camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
            for(HuskyLens.Block block: camera.blocks()) {
                telemetry.addData("object:", block.id);
                telemetry.addData("x", block.x);
                telemetry.addData("y", block.y);
                telemetry.addData("width", block.width);
                telemetry.addData("height", block.height);
                telemetry.addLine();
            }
            sleep(30);
            telemetry.update();
        }
    }

    public void fixedAngleCameraTest() {
        FixedAngleArilTagCamera camera = getHuskyWithDefaultConfig();

        waitForStart();
        camera.init();

        while (!isStopRequested() && opModeIsActive()) {
            camera.periodic(0);
            for (FixedAngleArilTagCamera.AprilTagTarget target: camera.getArilTagTargets()) {
                telemetry.addData("id", target.id);
                telemetry.addData("position", target.getRelativePositionToRobot());
                telemetry.addLine();
            }
            telemetry.update();
            sleep(30);
        }
    }

    private void fixedAngleCameraWallDetectionTest() {
        FixedAngleArilTagCamera camera = getHuskyWithDefaultConfig();

        waitForStart();
        camera.init();

        while (!isStopRequested() && opModeIsActive()) {
            camera.periodic();
            for (String messageKey:camera.getDebugMessages().keySet())
                telemetry.addData(messageKey, camera.getDebugMessages().get(messageKey));
            telemetry.update();
            sleep(30);
        }
    }

    private void driveToAprilTagTest() {
        Chassis chassis = getChassisModuleWithDefaultConfig();

        waitForStart();

        chassis.setWheelSpeedControlEnabled(false, null);
        while (!isStopRequested() && opModeIsActive()) {
            updateRobot();

            if (gamepad1.left_bumper)
                chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                        Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_VISUAL,
                        new Vector2D(new double[]{0, -35})
                ), null);
            else
                chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                        Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY,
                        new Vector2D()
                ), null);

            if (gamepad1.right_bumper)
                chassis.setRotationalTask(new Chassis.ChassisRotationalTask(
                        Chassis.ChassisRotationalTask.ChassisRotationalTaskType.FACE_NAVIGATION_REFERENCES,
                        0 // default rotational value
                ), null);
            else
                chassis.setRotationalTask(new Chassis.ChassisRotationalTask(
                        Chassis.ChassisRotationalTask.ChassisRotationalTaskType.SET_ROTATIONAL_SPEED,
                        0
                ), null);
        }
    }

    private void conceptDriveToAprilTagBezierCurve() {
        Chassis chassis = getChassisModuleWithDefaultConfig();

        waitForStart();

        /**
         * 0 for unused
         * 1 for proceeding going to position
         * */
        int statusCode = 0;
        final double smoothOutPercentage = 0.5, maxVelocity = 60;
        final Vector2D targetedPosition = new Vector2D(new double[] {0, -20});
        BezierCurve currentBezierCurve = null;
        double currentBezierCurveTimeScale = 0;
        long timeCurrentPathStarted = 0;
        while (!isStopRequested() && opModeIsActive()) {
            chassis.gainOwnerShip(null);
            chassis.setRotationalTask(new Chassis.ChassisRotationalTask(Chassis.ChassisRotationalTask.ChassisRotationalTaskType.SET_ROTATIONAL_SPEED, 0), null);
            updateRobot();
            switch (statusCode) {
                case 0: {
                    chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY, new Vector2D()),null);
                    if (gamepad1.left_bumper) {
                        if (!chassis.isVisualNavigationAvailable()) break;
                        Vector2D currentPositionToWall = chassis.getRelativeFieldPositionToWall();
                        Vector2D differenceToTarget = Vector2D.displacementToTarget(currentPositionToWall, targetedPosition);
                        if (differenceToTarget.getMagnitude() < 3) break;
                        currentBezierCurveTimeScale = differenceToTarget.getMagnitude() / maxVelocity;
                        final double xDifference = differenceToTarget.getX(), yDifference = differenceToTarget.getY();
                        currentBezierCurve = new BezierCurve(
                                currentPositionToWall,
                                currentPositionToWall.addBy(new Vector2D(0, xDifference).multiplyBy(smoothOutPercentage)),
                                targetedPosition.addBy(new Vector2D(Math.PI * 3/2, yDifference).multiplyBy(smoothOutPercentage)),
                                targetedPosition
                        );
                        telemetry.addData("pt1", currentPositionToWall);
                        telemetry.addData("pt2", currentPositionToWall.addBy(differenceToTarget.multiplyBy(smoothOutPercentage)));
                        telemetry.addData("pt3", targetedPosition.addBy(differenceToTarget.multiplyBy(-smoothOutPercentage)));
                        telemetry.addData("pt4", targetedPosition);
                        telemetry.update();

                        chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_VISUAL, currentPositionToWall), null);
                        timeCurrentPathStarted = System.currentTimeMillis();
                        statusCode = 1;
                    }
                    break;
                }
                case 1: {
                    if (!gamepad1.left_bumper) {
                        statusCode = 0;
                        break;
                    }
                    double t = System.currentTimeMillis() - timeCurrentPathStarted; t/=1000;
                    chassis.updateDesiredTranslationInVisualNavigation(currentBezierCurve.getPositionWithLERP(t / currentBezierCurveTimeScale), null);
                    break;
                }
            }
        }
    }

    private void fixedAngleAprilTagCameraVerticalParameterMeasuring() {
        HuskyAprilTagCamera huskyAprilTagCamera = new HuskyAprilTagCamera(hardwareMap.get(HuskyLens.class, "husky"));
        huskyAprilTagCamera.startRecognizing();
        waitForStart();

        FixedAngleCameraProfile.measureCameraVerticalParams(
                new HuskyAprilTagCamera(hardwareMap.get(HuskyLens.class, "husky")),
                telemetry,
                gamepad1,
                24,
                60,
                20,
                5
        );

        // results:
        final double cameraAngleRadianPerPixel = -0.003432;
        final double cameraInstallationAngleRadian = 0.955;
    }

    private void fixedAngleAprilTagCameraHorizontalParameterMeasuring() {
        HuskyAprilTagCamera huskyAprilTagCamera = new HuskyAprilTagCamera(hardwareMap.get(HuskyLens.class, "husky"));
        huskyAprilTagCamera.startRecognizing();
        waitForStart();

        FixedAngleCameraProfile.measureCameraHorizontalParams(
                new HuskyAprilTagCamera(hardwareMap.get(HuskyLens.class, "husky")),
                telemetry,
                gamepad1,
                new double[]{30, 40, 50},
                15, 5
        );

        // result: -0.00314, r^2 = 0.9903
    }

    private void encoderDriveToPositionTest() {
        Chassis chassis = getChassisModuleWithDefaultConfig();

        waitForStart();

        chassis.setWheelSpeedControlEnabled(false, null);
        while (!isStopRequested() && opModeIsActive()) {
            Chassis.ChassisTranslationalTask translationalTask;
            if (gamepad1.y)
                translationalTask = new Chassis.ChassisTranslationalTask(
                        Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                        new Vector2D(new double[]{20, 30})
                );
            else if (gamepad1.a)
                translationalTask = new Chassis.ChassisTranslationalTask(
                        Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                        new Vector2D(new double[]{-20, -30})
                );
            else
                translationalTask = new Chassis.ChassisTranslationalTask(
                        Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY,
                        new Vector2D()
                );
            chassis.setTranslationalTask(translationalTask, null);
            chassis.setRotationalTask(
                    new Chassis.ChassisRotationalTask(
                        Chassis.ChassisRotationalTask.ChassisRotationalTaskType.SET_ROTATIONAL_SPEED,
                            (Math.abs(gamepad1.left_stick_x) > 0.05) ? -gamepad1.left_stick_x : 0
                    ),
                    null
            );

            updateRobot();
        }
    }

    private void conceptAutoStageTest() {
        Chassis chassis = getChassisModuleWithDefaultConfig();
        List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        long t0 = System.currentTimeMillis();
        BezierCurve path = new BezierCurve(
                new Vector2D(new double[] {0, 0}),
                new Vector2D(new double[] {0, 50}),
                new Vector2D(new double[] {150, 100}),
                new Vector2D(new double[] {200, 100})
        );
        telemetry.addData("time for path cal", System.currentTimeMillis() - t0);
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> path,
                () -> {
                    telemetry.addLine("start at segment");
                },
                () -> {
                    telemetry.addLine("auto service periodic");
                },
                () -> {
                    telemetry.addLine("auto service end");
                    telemetry.update();
                    sleep(1000);
                },
                () -> true,
                () -> new Rotation2D(0), () -> new Rotation2D(Math.PI / 2)
        ));
        AutoProgramRunner autoProgramRunner = new AutoProgramRunner(chassis);

        autoProgramRunner.scheduleCommandSegments(commandSegments);

        waitForStart();
        t0 = System.currentTimeMillis();
        autoProgramRunner.init();
        telemetry.addData("time init", System.currentTimeMillis() - t0);
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {
            autoProgramRunner.periodic();
            updateRobot();
            if (autoProgramRunner.isCurrentSegmentComplete())
                break;
        }
    }

    private void tofDistanceSensorTest() {
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("distance sensor reading", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            sleep(20);
        }
    }

    private void lightingTest() {
        DcMotor lighting = hardwareMap.get(DcMotor.class, "lighting");

        waitForStart();

        double lightingPower = 0.75;
        while (!isStopRequested() && opModeIsActive()) {
            lighting.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (gamepad1.a)
               lighting.setPower(lightingPower);
            else
                lighting.setPower(0);

            if (Math.abs(gamepad1.left_stick_y) > 0.05)
                lightingPower += -50.0f/1000.0f * gamepad1.left_stick_y;

            telemetry.addData("lighting pow", lightingPower);
            telemetry.update();

            if (lightingPower < 0) lightingPower = 0;
            else if (lightingPower > 0.9) lightingPower = 0.9;
            sleep(50);
        }
    }

    private void conceptServoSync() {
        // moving two parallel and adjacent servos together
        final Servo servo1 = hardwareMap.get(Servo.class, "servo1"), servo2 = hardwareMap.get(Servo.class, "servo2");
        final boolean servo1Reversed = false, servo2Reversed = true;

        waitForStart();
        double desiredPosition = 0;
        boolean enabled = true, wasAPressed = false;
        while (!isStopRequested() && opModeIsActive()) {
            if (Math.abs(gamepad1.left_stick_y) > 0.05)
                desiredPosition += -50.0f / 1000.0f * 0.5 * gamepad1.left_stick_y;
            if (desiredPosition > 1) desiredPosition = 1;
            else if (desiredPosition < 0) desiredPosition = 0;
            if ((!wasAPressed) && gamepad1.a)
                enabled = !enabled;
            wasAPressed = gamepad1.a;

            servo1.setPosition(servo1Reversed ? 1-desiredPosition : desiredPosition);
            servo2.setPosition(servo2Reversed ? 1-desiredPosition : desiredPosition);

            telemetry.addData("desired position", desiredPosition);
            telemetry.addData("enabled", enabled);
            telemetry.update();
            sleep(50);
        }
    }

    private void tensorCameraTest() {
        TensorCamera testCamera = new TensorCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        testCamera.startRecognizing();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("FPS", testCamera.getCameraRefreshRate());
            int id = 0;
            for (RawPixelDetectionCamera.PixelTargetRaw pixelTarget:testCamera.getPixelTargets())
                telemetry.addData("pixel target" + (++id), pixelTarget); // nice and working

            telemetry.update();
        }

        testCamera.stopRecognizing();
    }

    private void pixelCameraVerticalParamMeasuring() {
        TensorCamera testCamera = new TensorCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        testCamera.startRecognizing();
        FixedAngleCameraProfile.measureCameraVerticalParams(testCamera, telemetry, gamepad1, 20, 34, 14);
        /*
        * result:
        * camera radian per pixel:  -9.19E-4
        * camera installation angle radian: 0.784
        * r^2: 0.995
        * */
    }

    private void pixelCameraHorizontalParamMeasuring() {
        TensorCamera testCamera = new TensorCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        testCamera.startRecognizing();
         FixedAngleCameraProfile.measureCameraHorizontalParams(testCamera, telemetry, gamepad1, new double[] {15, 20, 25}, 12);
         /*
         * result: camera radian per pixel 0.001092
         *  */
    }

    private void fixedAnglePixelDetectionCameraTest() {
        TensorCamera tensorCamera = hardwareMap.get(TensorCamera.class, "Webcam 1");
        FixedAnglePixelCamera pixelCamera = getPixelCameraWithDefaultConfig();

        pixelCamera.init();
        waitForStart();

        pixelCamera.enableCamera();

        while (!isStopRequested() && opModeIsActive()) {
            pixelCamera.periodic();
            telemetry.addData("fps", tensorCamera.getCameraRefreshRate());
            final Vector2D pos = pixelCamera.getNearestPixelPosition();
            telemetry.addData("nearest pixel relative position to robot", pos!=null ? pos : "unseen");
            telemetry.update();
            sleep(50);
        }
    }

    private void limitSwitchTest() {
        TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "limit");

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("limit switch status", limitSwitch.isPressed());
            telemetry.update();

            sleep(50);
        }
    }

    private void driveToPixelAimBotTest() {
        Chassis chassis = getChassisModuleWithDefaultConfig();
        FixedAnglePixelCamera pixelCamera = getPixelCameraWithDefaultConfig();
        telemetrySender.addRobotModule(pixelCamera);
        PixelCameraAimBot aimBot = new PixelCameraAimBot(chassis, pixelCamera, null, new HashMap<>());

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            pixelCamera.periodic();
            if (gamepad1.a)
                aimBot.initiateAim(PixelCameraAimBot.AimMethod.FACE_TO_AND_FEED);
            else if (gamepad1.b)
                aimBot.initiateAim(PixelCameraAimBot.AimMethod.LINE_UP_AND_FEED);
            if (gamepad1.left_bumper)
                aimBot.update();
            else {
                chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY, new Vector2D()), null);
                chassis.setRotationalTask(new Chassis.ChassisRotationalTask(Chassis.ChassisRotationalTask.ChassisRotationalTaskType.SET_ROTATIONAL_SPEED, 0), null);
            }
            updateRobot();

            sleep(20);
        }
    }

    private void newArmTest() {
        DcMotor arm1 = hardwareMap.get(DcMotor.class, "arm1"), arm2 = hardwareMap.get(DcMotor.class, "arm2");

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            final double arm1Power = Math.abs(gamepad1.left_stick_y) > 0.05 ? -gamepad1.left_stick_y:0,
            arm2Power = Math.abs(gamepad1.right_stick_y) > 0.05 ? -gamepad1.right_stick_y:0;

            arm1.setPower(arm1Power);
            arm2.setPower(arm2Power);

            telemetry.addData("arm1 pow", arm1Power);
            telemetry.addData("arm2 pow", arm2Power);
            telemetry.update();
        }
    }
}
