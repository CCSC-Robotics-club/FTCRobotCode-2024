package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.EncoderMotorWheel;
import org.firstinspires.ftc.teamcode.Modules.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Modules.TripleIndependentEncoderAndIMUPositionEstimator;
import org.firstinspires.ftc.teamcode.Services.AutoProgramRunner;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.FixedAngleCameraProfile;
import org.firstinspires.ftc.teamcode.Utils.EnhancedPIDController;
import org.firstinspires.ftc.teamcode.Utils.HuskyAprilTagCamera;
import org.firstinspires.ftc.teamcode.Utils.RawPixelDetectionCamera;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.SimpleFeedForwardSpeedController;
import org.firstinspires.ftc.teamcode.Utils.TensorCamera;
import org.firstinspires.ftc.teamcode.Utils.Vector2D;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Test_main")
public class TestMain extends LinearOpMode {
    @Override
    public void runOpMode() {
       encoderParamsMeasuring();
    }

    List<RobotModule> robotModules = new ArrayList<>(1);
    TelemetrySender telemetrySender;
    private Chassis getChassisModuleWithDefaultConfig() {
        FixedAngleArilTagCamera camera = getHuskyWithDefaultConfig();


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

        TripleIndependentEncoderAndIMUPositionEstimator positionEstimator = new TripleIndependentEncoderAndIMUPositionEstimator(
                hardwareMap.get(DcMotor.class, "backRight"),
                hardwareMap.get(DcMotor.class, "frontRight"),
                hardwareMap.get(DcMotor.class, "backLeft"),
                imu,
                RobotConfig.testConfig.encodersParams
        );
        positionEstimator.init();
        robotModules.add(positionEstimator);

        Chassis chassis = new Chassis(frontLeftWheel, frontRightWheel, backLeftWheel ,backRightWheel, positionEstimator, camera, FixedAngleArilTagCamera.WallTarget.Name.RED_ALLIANCE_WALL);

        EnhancedPIDController.StaticPIDProfile visualPIDControllerProfile = new EnhancedPIDController.StaticPIDProfile(
                Double.POSITIVE_INFINITY,
                0.6,
                0.07,
                25,
                3,
                0.15,
                0,0
        );
        EnhancedPIDController visualPIDControllerX = new EnhancedPIDController(visualPIDControllerProfile),
                visualPIDControllerY = new EnhancedPIDController(visualPIDControllerProfile);

        camera.init();
        frontRightWheel.init();
        frontLeftWheel.init();
        backRightWheel.init();
        backLeftWheel.init();
        chassis.init();


        robotModules.add(camera);
        robotModules.add(frontLeftWheel);
        robotModules.add(frontRightWheel);
        robotModules.add(backLeftWheel);
        robotModules.add(backRightWheel);
        robotModules.add(chassis);

        this.telemetrySender = new TelemetrySender(telemetry);
        telemetrySender.init();

        telemetrySender.addRobotModule(chassis);
        telemetrySender.addRobotModule(positionEstimator);
        telemetrySender.addRobotModule(camera);

        chassis.gainOwnerShip(null);
        return chassis;
    }

    private FixedAngleArilTagCamera getHuskyWithDefaultConfig() {
        return new FixedAngleArilTagCamera(new HuskyAprilTagCamera(hardwareMap.get(HuskyLens.class, "husky")), RobotConfig.VisualNavigationConfigs.visualCameraProfile);
    }

    private void updateRobotModules() {
        for (RobotModule robotModule : robotModules)
            robotModule.periodic();
        telemetrySender.periodic();
    }

    private void portMatching() {
        DcMotor port0 = hardwareMap.get(DcMotor.class, "port0"),
                port1 = hardwareMap.get(DcMotor.class, "port1"),
                port2 = hardwareMap.get(DcMotor.class, "port2"),
                port3 = hardwareMap.get(DcMotor.class, "port3");

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addLine("press A to turn port0");
            telemetry.addLine("press B to turn port1");
            telemetry.addLine("press X to turn port2");
            telemetry.addLine("press Y to turn port3");
            telemetry.update();
            if (gamepad1.a)
                port0.setPower(0.5);
            else port0.setPower(0);

            if (gamepad1.b)
                port1.setPower(0.5);
            else port1.setPower(0);

            if (gamepad1.x)
                port2.setPower(0.5);
            else port2.setPower(0);

            if (gamepad1.y)
                port3.setPower(0.5);
            else port3.setPower(0);
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
        Servo servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");

        servo2.setDirection(Servo.Direction.REVERSE);

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
        DcMotor motor1 = hardwareMap.get(DcMotor.class, "backRight");

        EnhancedPIDController armController = new EnhancedPIDController(
                new EnhancedPIDController.StaticPIDProfile(
                        Double.POSITIVE_INFINITY,
                        0.75,
                        0.14,
                        Math.toRadians(18),
                        Math.toRadians(1),
                        0,
                        0,
                        0
                )
        );
        waitForStart();
        final double encoderValuePerRadian = -26000 / (Math.PI*20);
        double previousTimeMillis = System.currentTimeMillis();
        int encoderZeroPosition = 0;
        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.a) {
                armController.startNewTask(new EnhancedPIDController.Task(EnhancedPIDController.Task.TaskType.GO_TO_POSITION, Math.toRadians(40)));
                double correctionPower = armController.getMotorPower((motor1.getCurrentPosition()-encoderZeroPosition) / encoderValuePerRadian, (System.currentTimeMillis() - previousTimeMillis) / 1000.0f);
                telemetry.addData("correction power", correctionPower);
                motor1.setPower(correctionPower);
            } else if (gamepad1.b) {
                encoderZeroPosition = motor1.getCurrentPosition();
            } else {
                motor1.setPower(0);
            }
            telemetry.addData("encoder reading:", Math.toDegrees((motor1.getCurrentPosition() - encoderZeroPosition) / encoderValuePerRadian));
            telemetry.addData("raw:", motor1.getCurrentPosition());
            telemetry.update();
            previousTimeMillis = System.currentTimeMillis();
        }
        motor1.setPower(0);
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
        IMU imu = hardwareMap.get(IMU.class, "alternativeIMU");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot( // expansion hub
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                ));

        waitForStart();
        imu.resetYaw();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("imu yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.update();
        }
    }

    /**
     * measurement for EncoderBiasPerRadian
     */
    private void encoderParamsMeasuring() {
        DcMotorEx verticalEncoder1 = hardwareMap.get(DcMotorEx.class, "backRight"); // vertical 1, not reversed
        DcMotorEx verticalEncoder2 = hardwareMap.get(DcMotorEx.class, "frontLeft"); // vertical 2, reversed
        DcMotorEx horizontalEncoder = hardwareMap.get(DcMotorEx.class, "frontRight"); // horizontal, reversed

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
                new boolean[] {true, false, true},
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                gamepad1,
                telemetry
        );

        TripleIndependentEncoderAndIMUPositionEstimator.TripleIndependentEncoderAndIMUSystemParams tripleIndependentEncoderAndIMUSystemParams = new TripleIndependentEncoderAndIMUPositionEstimator.TripleIndependentEncoderAndIMUSystemParams(
                true,
                false,
                true,
                135.812,
                0.2049
        );
    }

    public void huskyTest() {
        HuskyLens camera = hardwareMap.get(HuskyLens.class, "husky");
        camera.initialize();
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
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
            updateRobotModules();

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
            updateRobotModules();
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
                14,
                60,
                20,
                4
        );

        // results:
        final double cameraAngleRadianPerPixel = -0.00295;
        final double cameraInstallationAngleRadian = 1.05986;
    }

    private void fixedAngleAprilTagCameraHorizontalParameterMeasuring() {
        HuskyAprilTagCamera huskyAprilTagCamera = new HuskyAprilTagCamera(hardwareMap.get(HuskyLens.class, "husky"));
        huskyAprilTagCamera.startRecognizing();
        waitForStart();

        FixedAngleCameraProfile.measureCameraHorizontalParams(
                new HuskyAprilTagCamera(hardwareMap.get(HuskyLens.class, "husky")),
                telemetry,
                gamepad1,
                new double[]{40, 50, 60},
                20,
                4
        );

        // results:
        final double cameraAngleRadianPerPixel = -0.003366;
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

            updateRobotModules();
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
                path,
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
                0, Math.PI / 2
        ));
        AutoProgramRunner autoProgramRunner = new AutoProgramRunner(commandSegments, chassis, telemetry);

        waitForStart();
        t0 = System.currentTimeMillis();
        autoProgramRunner.init();
        telemetry.addData("time init", System.currentTimeMillis() - t0);
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {
            autoProgramRunner.periodic();
            updateRobotModules();
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
        final boolean servo1Reversed = false, servo2Reversed = false;
        final double servo1ZeroPosition = 0, servo2ZeroPosition = 0;

        waitForStart();
        double desiredPosition = 0;
        boolean enabled = true, wasAPressed = false;
        servo1.setDirection(servo1Reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo2.setDirection(servo2Reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        while (!isStopRequested() && opModeIsActive()) {
            if (Math.abs(gamepad1.left_stick_y) > 0.05)
                desiredPosition += -50.0f / 1000.0f * 0.5 * gamepad1.left_stick_y;
            if (desiredPosition > 1) desiredPosition = 1;
            else if (desiredPosition < 0) desiredPosition = 0;
            if ((!wasAPressed) && gamepad1.a)
                enabled = !enabled;
            wasAPressed = gamepad1.a;

            servo1.setPosition(desiredPosition + servo1ZeroPosition);
            servo2.setPosition(desiredPosition + servo2ZeroPosition);

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
                telemetry.addData("pixel target" + (++id), pixelTarget);

            telemetry.update();
        }

        testCamera.stopRecognizing();
    }
}
