package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMUNew;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.FixedAnglePixelCamera;
import org.firstinspires.ftc.teamcode.Modules.TripleIndependentEncoderAndIMUPositionEstimator;
import org.firstinspires.ftc.teamcode.Services.AutoProgramRunner;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.RectangularRegionColorComparisonPipeLine;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedEncoder;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedIMU;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.Claw;
import org.firstinspires.ftc.teamcode.Utils.DualServoClaw;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.FixedAngleCameraProfile;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.HuskyAprilTagCamera;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.ArmGravityController;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.EncoderMotorMechanism;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.PixelCameraAimBotLegacy;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.RawObjectDetectionCamera;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.SimpleArmController;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.SingleServoClaw;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.TeamElementFinderTensorflow;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.TensorCamera;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Scanner;

@TeleOp(name="Test_main")
public class TestMain extends LinearOpMode {
    @Override
    public void runOpMode() {
        testColorTeamPropDetection();
    }

    private void robotRest() {
        String[] encoderNames = RobotConfig.competitionConfig.encoderNames;
        final DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        final DcMotor hor = hardwareMap.get(DcMotorEx.class, encoderNames[0]),
                ver1 = hardwareMap.get(DcMotorEx.class, encoderNames[0]),
                ver2 = hardwareMap.get(DcMotorEx.class, encoderNames[1]);
        final DcMotor climb0 = hardwareMap.get(DcMotor.class, "climbMotor0"),
                climb1 = hardwareMap.get(DcMotor.class, "climbMotor1");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("hor enc", hor.getCurrentPosition());
            telemetry.addData("ver 1 enc", ver1.getCurrentPosition());
            telemetry.addData("ver 2 enc", ver2.getCurrentPosition());
            telemetry.addData("dis back", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

            climb0.setPower(-gamepad1.left_stick_y);
            climb1.setPower(gamepad1.right_stick_y);
        }
    }

    private void ledTest() {
        final LED lightLeft = hardwareMap.get(LED.class, "lightLeft"),
                lightRight = hardwareMap.get(LED.class, "lightLeft");
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            lightRight.enable(gamepad1.a);
            lightLeft.enable(gamepad1.b);
        }
    }

    private void scoringSettingsTuning() {
        final DcMotor arm = hardwareMap.get(DcMotor.class, "arm"),
                extend = hardwareMap.get(DcMotor.class, "extend");
        final TouchSensor armLimit = hardwareMap.get(TouchSensor.class, "armLimit"),
                extendLimit = hardwareMap.get(TouchSensor.class, "extendLimit");
        final DistanceSensor distance = hardwareMap.get(DistanceSensor.class, "distance");

        Servo flip = hardwareMap.get(Servo.class, "flip");

        waitForStart();

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        final double extendEncoderFactor = RobotConfig.ExtendConfigs.extendEncoderReversed ? -1: 1,
                armEncoderFactor = RobotConfig.ArmConfigs.encoderReversed ? -1: 1;
        double extendEncoderZeroPosition = extend.getCurrentPosition(),
                armEncoderZeroPosition = arm.getCurrentPosition(),
                flipPosition = 0;
        while (!isStopRequested() && opModeIsActive()) {
            flipPosition -= 0.5*gamepad1.left_stick_y / 50;
            flipPosition = Math.max(flipPosition, 0);
            flipPosition = Math.min(flipPosition, 1);
            final double armPower = Math.abs(gamepad1.right_stick_y) > 0.05? -gamepad1.right_stick_y : 0;
            final double extendPower = gamepad1.y ? 0.3 : (gamepad1.a ? -0.3 : 0);

            extend.setPower(extendPower);
            arm.setPower(armPower);
            flip.setPosition(flipPosition);

            if (armLimit.isPressed()) armEncoderZeroPosition = arm.getCurrentPosition();
            if (extendLimit.isPressed()) extendEncoderZeroPosition = extend.getCurrentPosition();


            telemetry.addData("extend pow", extendPower);
            telemetry.addData("flip position", flipPosition);
            telemetry.addData("arm position", (arm.getCurrentPosition() - armEncoderZeroPosition) * armEncoderFactor);
            telemetry.addData("extend position", (extend.getCurrentPosition() - extendEncoderZeroPosition) * extendEncoderFactor);
            telemetry.addData("distance to wall", distance.getDistance(DistanceUnit.CM));

            telemetry.update();
            sleep(20);
        }
    }

    private void dualArmMotorTest() {
        final DcMotor arm1 = hardwareMap.get(DcMotor.class, "arm"), arm2 = hardwareMap.get(DcMotor.class, "arm2");

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            arm1.setPower(-gamepad1.left_stick_y);
            arm2.setPower(-gamepad1.right_stick_y);

            telemetry.addData("arm 1 pow", -gamepad1.left_stick_y);
            telemetry.addData("arm 2 pow", -gamepad1.right_stick_y);
            telemetry.addData("arm 1 enc reading", arm1.getCurrentPosition());
            telemetry.update();
        }
    }

    private void clawCalibration() {
        final Servo claw = hardwareMap.get(Servo.class, "clawLeft");

        waitForStart();

        double clawAngle = 0;
        while (opModeIsActive() && !isStopRequested()) {
            clawAngle -= gamepad1.left_stick_y * 0.5 / 50;
            clawAngle = Math.max(0, clawAngle);
            clawAngle = Math.min(clawAngle, 1);
            claw.setPosition(clawAngle);
            telemetry.addData("claw pos", clawAngle);
            telemetry.update();
            sleep(20);
        }
    }

    private void simpleArmTuning() {
        final DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        final TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "armLimit");
        final ThreadedEncoder armEncoder = new ThreadedEncoder(armMotor);
        armEncoder.update();
        final double armEncoderReadingFactor = RobotConfig.ArmConfigs.encoderReversed ? -1:1;
        double desiredPosition = 0, encoderZeroPosition = armEncoder.getSensorReading();
        final double armMotorRate = RobotConfig.ArmConfigs.motor1Reversed ? -1 : 1;
        waitForStart();

        final SimpleArmController armController = new SimpleArmController(
                RobotConfig.ArmConfigs.maxPowerWhenMovingUpNormal,
                RobotConfig.ArmConfigs.maxPowerWhenMovingDownNormal,
                RobotConfig.ArmConfigs.errorStartDecelerateNormal,
                RobotConfig.ArmConfigs.powerNeededToMoveUpNormal,
                RobotConfig.ArmConfigs.powerNeededToMoveDownNormal,
                RobotConfig.ArmConfigs.errorToleranceNormal,
                false
        );
        while (!isStopRequested() && opModeIsActive()) {
            armEncoder.update();
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (limitSwitch.isPressed())
                encoderZeroPosition = armEncoder.getSensorReading();

            final double armPosition = (armEncoder.getSensorReading() - encoderZeroPosition) * armEncoderReadingFactor;

            desiredPosition += gamepad1.right_stick_y * -20;
            desiredPosition = Math.max(0, desiredPosition);
            desiredPosition = Math.min(1500, desiredPosition);
            if (gamepad1.b)
                armController.desiredPosition = desiredPosition;

            final double commandedPower = Math.abs(gamepad1.left_stick_y) > 0.05 ? -gamepad1.left_stick_y:0,
                    correctionPower = armController.getMotorPower(armEncoder.getVelocity() * armEncoderReadingFactor, armPosition),
                    decidedPower = gamepad1.a ? correctionPower : commandedPower;
            armMotor.setPower(decidedPower * armMotorRate);

            telemetry.addData("commanded power", commandedPower);
            telemetry.addData("desired position (press b to send to controller)", desiredPosition);


            telemetry.addData("arm encoder reading", armPosition);
            telemetry.addData("arm velocity", armEncoder.getVelocity() * armEncoderReadingFactor);
            telemetry.addData("correction pow", correctionPower);

            telemetry.update();
            sleep(20);
        }
    }

    private void profiledArmTuning() {
        final DcMotorEx armMotor1 = hardwareMap.get(DcMotorEx.class, "arm");
        final TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "armLimit");
        final ThreadedEncoder armEncoder = new ThreadedEncoder(armMotor1);
        final ArmGravityController controller = new ArmGravityController(RobotConfig.ArmConfigs.armProfile);
        final double armEncoderReadingFactor = RobotConfig.ArmConfigs.encoderReversed ? -1:1;

        waitForStart();

        armEncoder.update();
        double desiredPosition = 0, encoderZeroPosition = armEncoder.getSensorReading();
        final double armMotor1Rate = RobotConfig.ArmConfigs.motor1Reversed ? -1 : 1;
        while (!isStopRequested() && opModeIsActive()) {
            armEncoder.update();
            armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (limitSwitch.isPressed())
                encoderZeroPosition = armEncoder.getSensorReading();

            final double armPosition = (armEncoder.getSensorReading() - encoderZeroPosition) * armEncoderReadingFactor;

            desiredPosition += gamepad1.right_stick_y * -20;
            desiredPosition = Math.max(0, desiredPosition);
            desiredPosition = Math.min(400, desiredPosition);
            if (gamepad1.b)
                controller.goToDesiredPosition(desiredPosition);

            final double commandedPower = Math.abs(gamepad1.left_stick_y) > 0.05 ? -gamepad1.left_stick_y:0,
                    correctionPower = controller.getMotorPower(armEncoder.getVelocity() * armEncoderReadingFactor, armPosition),
                    decidedPower = gamepad1.a ? correctionPower : commandedPower;
            armMotor1.setPower(decidedPower * armMotor1Rate);

            telemetry.addData("commanded power", commandedPower);
            telemetry.addData("desired position (press b to send to controller)", desiredPosition);


            telemetry.addData("arm encoder reading", armPosition);
            telemetry.addData("arm velocity", armEncoder.getVelocity() * armEncoderReadingFactor);
            telemetry.addData("correction pow", correctionPower);
            telemetry.addData("error accumulation", controller.getErrorAccumulation());

            telemetry.update();
            sleep(20);
        }
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

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        EncoderMotorMechanism frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel;
        frontLeftWheel = new EncoderMotorMechanism(frontLeftMotor);
        frontRightWheel = new EncoderMotorMechanism(frontRightMotor);
        backLeftWheel = new EncoderMotorMechanism(backLeftMotor);
        backRightWheel = new EncoderMotorMechanism(backRightMotor);

        frontLeftWheel.setEncoderReversed(RobotConfig.testConfig.frontLeftWheel_encoderReversed);
        frontRightWheel.setEncoderReversed(RobotConfig.testConfig.frontRightWheel_encoderReversed);
        backLeftWheel.setEncoderReversed(RobotConfig.testConfig.backLeftWheel_encoderReversed);
        backRightWheel.setEncoderReversed(RobotConfig.testConfig.backRightWheel_encoderReversed);

        frontLeftWheel.setMotorReversed(RobotConfig.testConfig.frontLeftWheel_motorReversed);
        frontRightWheel.setMotorReversed(RobotConfig.testConfig.frontRightWheel_motorReversed);
        backLeftWheel.setMotorReversed(RobotConfig.testConfig.backLeftWheel_motorReversed);
        backRightWheel.setMotorReversed(RobotConfig.testConfig.backRightWheel_motorReversed);

        RobotConfig.HardwareConfigs hardwareConfigs = RobotConfig.testConfig;
        String[] encoderNames =hardwareConfigs.encoderNames == null ?
                new String[] {"frontLeft", "frontRight", "backLeft"} :
                hardwareConfigs.encoderNames;
        final ThreadedEncoder horizontalEncoder = new ThreadedEncoder(hardwareMap.get(DcMotorEx.class, encoderNames[0])),
                verticalEncoder1 = new ThreadedEncoder(hardwareMap.get(DcMotorEx.class, encoderNames[1])),
                verticalEncoder2 = new ThreadedEncoder(hardwareMap.get(DcMotorEx.class, encoderNames[2]));
        final ThreadedIMU imuSensor = new ThreadedIMU(imu);


        TripleIndependentEncoderAndIMUPositionEstimator positionEstimator = new TripleIndependentEncoderAndIMUPositionEstimator(
                horizontalEncoder,
                verticalEncoder1,
                verticalEncoder2,
                imuSensor,
                hardwareConfigs.encodersParams
        );
        positionEstimator.init();
        robotModules.add(positionEstimator);

        Chassis chassis = new Chassis(frontLeftWheel, frontRightWheel, backLeftWheel ,backRightWheel, positionEstimator, null, FixedAngleArilTagCamera.WallTarget.Name.RED_ALLIANCE_WALL);

        // camera.init();
        chassis.init();


        // robotModules.add(camera);
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

    private void servoTest() {
        Servo servo = hardwareMap.get(Servo.class, "plane");
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
            servo.setPosition(servoAngle);
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
        Servo servo0 = hardwareMap.get(Servo.class, "climb0"), servo1 = hardwareMap.get(Servo.class, "climb1");

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            double power = -gamepad1.left_stick_y;
            if (Math.abs(power) < 0.05) power = 0;

            servo0.setPosition(-power * 0.5 + 0.5);
            servo1.setPosition(power * 0.5 + 0.5);

            telemetry.addData("pow", power);
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
        DcMotorEx verticalEncoder1 = hardwareMap.get(DcMotorEx.class, RobotConfig.competitionConfig.encoderNames[1]); // vertical 1
        DcMotorEx verticalEncoder2 = hardwareMap.get(DcMotorEx.class, RobotConfig.competitionConfig.encoderNames[2]); // vertical 2
        DcMotorEx horizontalEncoder = hardwareMap.get(DcMotorEx.class, RobotConfig.competitionConfig.encoderNames[0]); // horizontal

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
                new boolean[] {RobotConfig.competitionConfig.encodersParams.horizontalEncoderReversed, RobotConfig.competitionConfig.encodersParams.verticalEncoder1Reversed, RobotConfig.competitionConfig.encodersParams.verticalEncoder2Reversed},
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                gamepad1,
                telemetry,
                () -> isStopRequested() || !opModeIsActive()
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
        camera.restCamera();

        while (!isStopRequested() && opModeIsActive()) {
            camera.updateCamera();
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
        camera.restCamera();

        while (!isStopRequested() && opModeIsActive()) {
            camera.updateCamera();
            for (String messageKey:camera.getDebugMessages().keySet())
                telemetry.addData(messageKey, camera.getDebugMessages().get(messageKey));
            telemetry.update();
            sleep(30);
        }
    }

    private void driveToAprilTagTest() {
        Chassis chassis = getChassisModuleWithDefaultConfig();

        waitForStart();

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
                21,
                60,
                20,
                2
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

        autoProgramRunner.setCommandSegments(commandSegments);

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
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceBack");

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
            for (RawObjectDetectionCamera.PixelTargetRaw pixelTarget:testCamera.getPixelTargets())
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
        TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "armLimit");

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
        PixelCameraAimBotLegacy aimBot = new PixelCameraAimBotLegacy(chassis, pixelCamera, null, new HashMap<>());

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            pixelCamera.periodic();
            if (gamepad1.a)
                aimBot.initiateAim(PixelCameraAimBotLegacy.AimMethod.FACE_TO_AND_FEED);
            else if (gamepad1.b)
                aimBot.initiateAim(PixelCameraAimBotLegacy.AimMethod.LINE_UP_AND_FEED);
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

    private void armAndExtendTest() {
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm"),
                extend = hardwareMap.get(DcMotor.class, "extend");
        TouchSensor armLimit = hardwareMap.get(TouchSensor.class, "armLimit"),
                extendLimit = hardwareMap.get(TouchSensor.class, "extendLimit");

        waitForStart();

        final double extendEncoderFactor = RobotConfig.ExtendConfigs.extendEncoderReversed ? -1: 1,
                armEncoderFactor = RobotConfig.ArmConfigs.encoderReversed ? -1: 1;
        double extendEncoderZeroPosition = extend.getCurrentPosition(),
                armEncoderZeroPosition = arm.getCurrentPosition();
        while (!isStopRequested() && opModeIsActive()) {
            final double armPower = Math.abs(gamepad1.left_stick_y) > 0.05 ? -gamepad1.left_stick_y:0,
                    extendPower = Math.abs(gamepad1.right_stick_y) > 0.05 ? -gamepad1.right_stick_y:0;

            arm.setPower(armPower);
            extend.setPower(extendPower);

            if (armLimit.isPressed()) armEncoderZeroPosition = arm.getCurrentPosition();
            if (extendLimit.isPressed()) extendEncoderZeroPosition = extend.getCurrentPosition();

            telemetry.addData("arm pow", armPower);
            telemetry.addData("extend pow", extendPower);
            telemetry.addData("arm encoder reading", (arm.getCurrentPosition() - armEncoderZeroPosition) * armEncoderFactor);
            telemetry.addData("extend encoder reading", (extend.getCurrentPosition() - extendEncoderZeroPosition) * extendEncoderFactor);

            telemetry.update();
        }
    }

    private void conceptServoBasedArm() {

    }

    private void conceptDualClaw() {

    }

    private void testJsonFile() {
        waitForStart();
        try {
            InputStream is = hardwareMap.appContext.getAssets().open("deploy/pathplanner/paths/park.path");
            Scanner scanner = new Scanner(is).useDelimiter("\\A");
            String jsonContent = scanner.hasNext() ? scanner.next() : "";

            JSONObject pathJson = new JSONObject(jsonContent);
            JSONArray waypointsJson = pathJson.getJSONArray("waypoints");

            List<BezierCurve> curves = new ArrayList<>();
            for (int i = 0; i < waypointsJson.length() - 1; i++) {
                JSONObject point = (JSONObject) waypointsJson.get(i),
                        nextPoint = (JSONObject) waypointsJson.get(i+1);

                final JSONObject anchorPointJson = (JSONObject) point.get("anchor"), nextControlPointJson = (JSONObject) point.get("nextControl");
                telemetry.addData("anchor x", ((Number) anchorPointJson.get("x")).doubleValue());
                telemetry.addData("anchor y", ((Number) anchorPointJson.get("y")).doubleValue());
                telemetry.addData("next ctrl x", ((Number) nextControlPointJson.get("x")).doubleValue());
                telemetry.addData("next ctrl y", ((Number) nextControlPointJson.get("y")).doubleValue());
            }
        } catch (IOException e) {
            throw new RuntimeException("error while reading json file");
        } catch (JSONException e) {
            throw new RuntimeException("error while parsing json file");
        }

        telemetry.update();
        sleep(5000);
    }

    private void testLift() {
        final DcMotor lift = hardwareMap.get(DcMotor.class, "lift");
        hardwareMap.get(DigitalChannel.class, "dio1").setMode(DigitalChannel.Mode.INPUT);
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            lift.setPower(-gamepad1.left_stick_y);
        }
    }

    private void tensorFlowAndAprilTagCameraTest() {
        final String[] LABELS = {
                "team-prop-red",
        };
        final AprilTagProcessor aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        final TfodProcessor tfod = new TfodProcessor.Builder().setModelAssetName("Team Prop Red.tflite").setModelLabels(LABELS).build();
        tfod.setMinResultConfidence(0.75f);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.addProcessor(tfod);
        builder.addProcessor(aprilTag);

        final VisionPortal visionPortal = builder.build();
        visionPortal.setProcessorEnabled(tfod, true);
        visionPortal.setProcessorEnabled(aprilTag, false);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            }

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addData("tag id", detection.id);
                telemetry.addData("center x", detection.center.x);
                telemetry.addData("center y", detection.center.y);
            }

            telemetry.update();
        }
        tfod.shutdown();
    }

    private void armAndClawSync() {
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "arm");
        Servo flip = hardwareMap.get(Servo.class, "flip");
        TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "armLimit");

        waitForStart();

        final double powerRate = -1;
        double servoCurrentDesiredPosition = 0.5;
        int startingPos = armMotor.getCurrentPosition();
        while (!isStopRequested() && opModeIsActive()) {
            double power = gamepad1.left_stick_y * powerRate;

            servoCurrentDesiredPosition += gamepad1.right_stick_y * -0.05;
            servoCurrentDesiredPosition = Math.min(Math.max(0, servoCurrentDesiredPosition), 1);
            if (Math.abs(power) < 0.05)
                power = 0;

            armMotor.setPower(power);
            flip.setPosition(servoCurrentDesiredPosition);

            telemetry.addData("arm encoder reading", armMotor.getCurrentPosition() - startingPos);
            telemetry.addData("limit switch", limitSwitch.isPressed());
            telemetry.addData("flip servo position", servoCurrentDesiredPosition);

            if (limitSwitch.isPressed())
                startingPos = armMotor.getCurrentPosition();
            telemetry.update();
            sleep(50);
        }
    }

    private void teamElementFinderTest() {
        final TeamElementFinderTensorflow teamElementFinder = new TeamElementFinderTensorflow(hardwareMap, Robot.Side.RED);

        waitForStart();

        telemetry.addLine("finding team element...");
        telemetry.update();

        teamElementFinder.findTeamElementAndShutDown(5000);
        telemetry.addData("team element finder result: ", teamElementFinder.teamElementPosition);
        telemetry.update();
        while (opModeIsActive() && !isStopRequested()) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private void conceptColorSensorDetection() {
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "markSensor");

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            colorSensor.enableLed(gamepad1.a);
            /*
             * reading when facing ground: 30
             * reading when pixel detected: > 100
             * reading when sensor disconnected: 0
             *  */
            telemetry.addData("alpha", colorSensor.alpha());
            telemetry.addData("red", colorSensor.red());
            telemetry.addData("green", colorSensor.green());
            telemetry.addData("blue", colorSensor.blue());
            telemetry.update();
            sleep(50);
        }
    }

    private void conceptDigitalLight() {
        DigitalChannel light = hardwareMap.get(DigitalChannel.class, "light");
        light.setMode(DigitalChannel.Mode.OUTPUT);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            light.setState(gamepad1.a);
        }
    }

//    private void testTeamPropFinder() {
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCamera(webcamName);
//        builder.enableLiveView(true);
//        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
//
//        final VisionPortal visionPortal = builder.build();
//
//        waitForStart();
//
//
//        while (opModeIsActive() && !isStopRequested()) {
//            final double sensitivity = 5;
//            testPipeLine.regionOfInterest[0] += (int) (gamepad1.left_stick_x * sensitivity);
//            testPipeLine.regionOfInterest[1] -= (int) (gamepad1.left_stick_y * sensitivity);
//
//            testPipeLine.regionOfInterest[2] += (int) (gamepad1.right_stick_x * sensitivity);
//            testPipeLine.regionOfInterest[3] -= (int) (gamepad1.right_stick_y * sensitivity);
//
//            telemetry.addData("result: ", testPipeLine.result);
//            telemetry.update();
//            sleep(20);
//        }
//    }
//
//    class TestProcessor implements VisionProcessor {
//        public final int[] regionOfInterest = new int[] {0, 0, 0, 0};
//        Mat YCbCr = new Mat(), output = new Mat(), crop = new Mat();
//        private final Paint mPaint = new Paint();
//        double result = 0;
//
//        @Override
//        public void init(int i, int i1, CameraCalibration cameraCalibration) {
//            mPaint.setColor(Color.RED);
//            mPaint.setStyle(Paint.Style.STROKE);
//            mPaint.setStrokeWidth(5);
//        }
//
//        @Override
//        public Object processFrame(Mat inp, long l) {
//            Imgproc.cvtColor(inp, YCbCr, Imgproc.COLOR_RGB2YCrCb);
//            Rect regionOfInterest = new Rect(this.regionOfInterest[0], this.regionOfInterest[1], this.regionOfInterest[2], this.regionOfInterest[3]);
//            crop = YCbCr.submat(regionOfInterest);
//            Core.extractChannel(crop, crop, 2);
//
//            result = Core.mean(crop).val[0];
//            return null;
//        }
//
//        @Override
//        public void onDrawFrame(Canvas canvas, int i, int i1, float v, float v1, Object o) {
//            canvas.drawRect(this.regionOfInterest[0], this.regionOfInterest[1], this.regionOfInterest[2], this.regionOfInterest[3], mPaint);
//        }
//    }

    private void climbTest() {
        final Servo climb0 = hardwareMap.get(Servo.class, "climb0"),
                climb1 = hardwareMap.get(Servo.class, "climb1");

        final DcMotor climbMotor0 = hardwareMap.get(DcMotor.class, "climbMotor0"),
                climbMotor1 = hardwareMap.get(DcMotor.class, "climbMotor1");


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            final double position = gamepad1.a ? 1: 0.5;
            climb0.setPosition(1-position);
            climb1.setPosition(position);

            double power = -gamepad1.left_stick_y;
            if (Math.abs(power) < 0.1)
                power = 0;

            climbMotor0.setPower(power);
            climbMotor1.setPower(-power);
        }
    }

    private void testColorTeamPropDetection() {
        final int width = 320, height = 240;
        final int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );
        final OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId
        );
        final RectangularRegionColorComparisonPipeLine.RegionOfInterest[] ROIs = new RectangularRegionColorComparisonPipeLine.RegionOfInterest[] {
                new RectangularRegionColorComparisonPipeLine.RegionOfInterest(50, 50, 160-80, 120),
                new RectangularRegionColorComparisonPipeLine.RegionOfInterest(50, 50, 160, 60),
                new RectangularRegionColorComparisonPipeLine.RegionOfInterest(50, 50, 160+80, 120)
        };
        webcam.setPipeline(new RectangularRegionColorComparisonPipeLine(
                RectangularRegionColorComparisonPipeLine.ColorChannel.BLUE,
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

        int currentIndex = 0;

        while (!isStopRequested() && !isStarted()) {
            if (gamepad1.x)
                currentIndex = 0;
            else if (gamepad1.y)
                currentIndex = 1;
            else if (gamepad1.b)
                currentIndex = 2;

            final RectangularRegionColorComparisonPipeLine.RegionOfInterest ROI = ROIs[currentIndex];
            ROI.centerX += gamepad1.left_stick_x;
            ROI.centerX = Math.min(Math.max(ROI.centerX, 0), width);
            ROI.centerY += gamepad1.left_stick_y;
            ROI.centerY = Math.min(Math.max(ROI.centerY, 0), height);

            sleep(50);
        }
    }
}
