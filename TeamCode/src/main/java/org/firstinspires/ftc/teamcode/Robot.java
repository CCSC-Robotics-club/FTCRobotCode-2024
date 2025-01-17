package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Arm;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.ClimbAndPlane;
import org.firstinspires.ftc.teamcode.Modules.Extend;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Modules.FlippableDualClaw;
import org.firstinspires.ftc.teamcode.Modules.TripleIndependentEncoderAndIMUPositionEstimator;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.DriverGamePad;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.HuskyAprilTagCamera;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedEncoder;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedIMU;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedLED;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.EncoderMotorMechanism;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedMotor;
import org.firstinspires.ftc.teamcode.Utils.PositionEstimator;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ProfiledServo;
import org.firstinspires.ftc.teamcode.Utils.ProgramRunningStatusChecker;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.RobotService;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedSensor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

public abstract class Robot {
    public final Gamepad gamepad1, gamepad2;
    public final HardwareMap hardwareMap;
    protected final RobotConfig.HardwareConfigs hardwareConfigs;
    public final Telemetry telemetry;
    public ProgramRunningStatusChecker programRunningStatusChecker;
    protected DriverGamePad driverGamePad = null;

    protected final boolean useMultiThread;

    private final EncoderMotorMechanism frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel;
    public ThreadedSensor distanceSensor, colorLeft, colorRight;
    public Chassis chassis;

    public Arm arm;
    public Extend extend;
    public FlippableDualClaw claw;

    public ClimbAndPlane climbAndPlane;

    public PositionEstimator positionEstimator;
    public FixedAngleArilTagCamera aprilTagCamera;
    protected DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    protected final List<RobotModule> robotModules = new ArrayList<>(1);
    protected final List<RobotService> robotServices = new ArrayList<>(1);
    public TelemetrySender telemetrySender;
    protected IMU imu, alternativeIMU;
    private final Map<String, ThreadedSensor> sensors = new HashMap<>();
    private final List<ThreadedMotor> motors = new ArrayList<>();
    private final List<ProfiledServo> servos = new ArrayList<>();

    public enum Side {
        RED,
        BLUE
    }
    protected final Side side;
    public Robot(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry driverStationTelemetry, ProgramRunningStatusChecker checker, RobotConfig.HardwareConfigs hardwareConfigs, Side side) {
        this(gamepad2, gamepad1, hardwareMap, driverStationTelemetry, checker, hardwareConfigs, side, false);
    }
    public Robot(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry driverStationTelemetry, ProgramRunningStatusChecker checker, RobotConfig.HardwareConfigs hardwareConfigs, Side side, boolean debugModeEnabled) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.side = side;

        this.hardwareConfigs = hardwareConfigs;
        this.hardwareMap = hardwareMap;

        final MultipleTelemetry multipleTelemetry = new MultipleTelemetry(driverStationTelemetry, FtcDashboard.getInstance().getTelemetry());
        this.telemetry = RobotConfig.useFtcDashboardForTelemetry ? multipleTelemetry : driverStationTelemetry;

        this.programRunningStatusChecker = checker;

        this.useMultiThread = !debugModeEnabled;
        // this.useMultiThread = false;

        telemetrySender = new TelemetrySender(driverStationTelemetry);

        /* <-- chassis --> */
        this.frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        this.frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        this.backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        this.backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");

        this.imu = hardwareMap.get(IMU.class, "imu");
        this.distanceSensor = new ThreadedSensor(() -> hardwareMap.get(DistanceSensor.class, "distance").getDistance(DistanceUnit.CM));
        this.sensors.put("distance", distanceSensor);

        imu.initialize(this.hardwareConfigs.imuParameter);
        if (hardwareConfigs.alternativeIMUParameter != null) {
            this.alternativeIMU = hardwareMap.get(IMU.class, "alternativeIMU");
            alternativeIMU.initialize(hardwareConfigs.alternativeIMUParameter);
        }
        else
            this.alternativeIMU = null;


        configureChassisMotors();
        frontLeftWheel = new EncoderMotorMechanism(frontLeftMotor);
        frontRightWheel = new EncoderMotorMechanism(frontRightMotor);
        backLeftWheel = new EncoderMotorMechanism(backLeftMotor);
        backRightWheel = new EncoderMotorMechanism(backRightMotor);

        String[] encoderNames = this.hardwareConfigs.encoderNames == null ?
                new String[] {"frontLeft", "frontRight", "backLeft"} :
                this.hardwareConfigs.encoderNames;

        final ThreadedEncoder horizontalEncoder = new ThreadedEncoder(hardwareMap.get(DcMotorEx.class, encoderNames[0])),
                verticalEncoder1 = new ThreadedEncoder(hardwareMap.get(DcMotorEx.class, encoderNames[1])),
                verticalEncoder2 = new ThreadedEncoder(hardwareMap.get(DcMotorEx.class, encoderNames[2]));
        final ThreadedIMU imuSensor = new ThreadedIMU(imu);
        sensors.put("horizontal-encoder", horizontalEncoder);
        sensors.put("vertical-encoder-1", verticalEncoder1);
        sensors.put("vertical-encoder-2", verticalEncoder2);
        sensors.put("imu", imuSensor);


        this.positionEstimator = new TripleIndependentEncoderAndIMUPositionEstimator(
                horizontalEncoder,
                verticalEncoder1,
                verticalEncoder2,
                imuSensor,
                this.hardwareConfigs.encodersParams
        );
        ((RobotModule) positionEstimator).init();
        robotModules.add((RobotModule) positionEstimator);

        configureChassisWheels();

        aprilTagCamera = new FixedAngleArilTagCamera(
                new HuskyAprilTagCamera(hardwareMap.get(HuskyLens.class, "husky")),
                RobotConfig.VisualNavigationConfigs.visualCameraProfile
        );

        chassis = new Chassis(frontLeftWheel, frontRightWheel, backLeftWheel ,backRightWheel, positionEstimator, aprilTagCamera,
                this.side == Side.RED ? FixedAngleArilTagCamera.WallTarget.Name.RED_ALLIANCE_WALL : FixedAngleArilTagCamera.WallTarget.Name.BLUE_ALLIANCE_WALL);
        robotModules.add(chassis);

        /* claw */
        final ProfiledServo flip = new ProfiledServo(hardwareMap.get(Servo.class, "flip"), 1),
                clawLeft = new ProfiledServo(hardwareMap.get(Servo.class, "clawLeft"), 1),
                clawRight = new ProfiledServo(hardwareMap.get(Servo.class, "clawRight"), 1);
        final ThreadedLED indicatorLightLeft = new ThreadedLED(hardwareMap.get(LED.class, "lightLeft")),
                indicatorLightRight = new ThreadedLED(hardwareMap.get(LED.class, "lightRight"));
        colorLeft = new ThreadedSensor(() -> hardwareMap.get(ColorSensor.class, "colorLeft").alpha());
        colorRight = new ThreadedSensor(() -> hardwareMap.get(ColorSensor.class, "colorRight").alpha());
        servos.add(flip);
        servos.add(clawLeft);
        servos.add(clawRight);
        sensors.put("color-left", colorLeft);
        sensors.put("color-right", colorRight);
        claw = new FlippableDualClaw(
                flip, clawLeft, clawRight,
                colorLeft, colorRight,
                indicatorLightLeft, indicatorLightRight
        );
        robotModules.add(claw);

        /* arm */
        final ThreadedMotor armMotor1 = new ThreadedMotor(hardwareMap.get(DcMotor.class, "arm"));
        final ThreadedEncoder armEncoder = new ThreadedEncoder(hardwareMap.get(DcMotorEx.class, "arm"));
        final ThreadedSensor armLimit = new ThreadedSensor(() -> hardwareMap.get(TouchSensor.class, "armLimit").isPressed() ? 1:0);
        motors.add(armMotor1);
        sensors.put("arm-enc", armEncoder);
        sensors.put("arm-lim", armLimit);
        arm = new Arm(armMotor1, armMotor1, armEncoder, armLimit);
        robotModules.add(arm);

        /* extend */
        final ThreadedMotor extendMotor = new ThreadedMotor(hardwareMap.get(DcMotor.class, "extend"));
        final ThreadedEncoder extendEncoder = new ThreadedEncoder(hardwareMap.get(DcMotorEx.class, "extend"));
        final ThreadedSensor extendLimit = new ThreadedSensor(() -> hardwareMap.get(TouchSensor.class, "extendLimit").isPressed() ? 1:0);
        motors.add(extendMotor);
        sensors.put("extend-enc", extendEncoder);
        sensors.put("extend-lim", extendLimit);
        extend = new Extend(extendMotor, extendEncoder, extendLimit);
        robotModules.add(extend);

        /* climb */
        climbAndPlane = new ClimbAndPlane(
                hardwareMap.get(Servo.class, "climb0"),
                hardwareMap.get(Servo.class, "climb1"),
                hardwareMap.get(Servo.class, "plane"),
                hardwareMap.get(DcMotor.class, "climbMotor0"),
                hardwareMap.get(DcMotor.class, "climbMotor1")
        );
        robotModules.add(climbAndPlane);
    }

    /**
     * initializes the robot
     */
    Thread updateSensorsThread, updateMotorsThread;
    public void initializeRobot() {
        /* <-- start of program --> */
        initModulesAndService();

        updateSensorsThread = new Thread(this::updateSensorsForever);
        updateMotorsThread = new Thread(this::updateMotorsForever);

        telemetry.addLine("startup complete...");
        telemetry.update();
    }

    public void startRobot() {
        if (!this.useMultiThread) return;

        updateSensorsThread.start();
        updateMotorsThread.start();
    }

    private long previousUpdateTimeMillis = System.currentTimeMillis();
    public void updateRobot() {
        if (!useMultiThread) {
            flushMotorsAndServos();
            flushSensorsAndControllers();
        }

        long t0 = System.currentTimeMillis();
        if (driverGamePad != null) driverGamePad.update();
        telemetrySender.putSystemMessage("controller update time(ms)", System.currentTimeMillis() - t0);

        t0 = System.currentTimeMillis();
        for (RobotService robotService: robotServices)
            robotService.periodic();
        telemetrySender.putSystemMessage("robot service update time(ms)", System.currentTimeMillis() - t0);

        for (RobotModule robotModule: robotModules) {
            t0 = System.currentTimeMillis();
            robotModule.periodic();
            telemetrySender.putSystemMessage("module <" + robotModule + "> update time", System.currentTimeMillis() - t0);
        }

        telemetrySender.periodic();

        while (System.currentTimeMillis() - previousUpdateTimeMillis < 9) {
            try {Thread.sleep(2);} catch (InterruptedException e) {throw new RuntimeException(e);}
        }

        previousUpdateTimeMillis = System.currentTimeMillis();
    }

    private long previousTimeMillis = System.currentTimeMillis();

    public void flushSensorsAndControllers() {
        aprilTagCamera.updateCamera();

        long t0 = System.currentTimeMillis();
        for (String sensorName: sensors.keySet()) {
            long t = System.currentTimeMillis();
            Objects.requireNonNull(sensors.get(sensorName)).update();
            long dt = System.currentTimeMillis() - t;
            if (dt > 10)
                telemetrySender.putSystemMessage("sensor <" + sensorName + "> update time (ms)", dt);
            else
                telemetrySender.deleteSystemMessage("sensor <" + sensorName + "> update time (ms)");
        }
        telemetrySender.putSystemMessage("sensors total update time(ms)", System.currentTimeMillis() - t0);
    }

    private void flushMotorsAndServos() {
        long t0 = System.currentTimeMillis();
        for (ThreadedMotor motor:motors)
            motor.update();
        telemetrySender.putSystemMessage("motors update time (ms)", System.currentTimeMillis() - t0);

        t0 = System.currentTimeMillis();
        frontLeftWheel.updateWithController(0, 0);
        frontRightWheel.updateWithController(0, 0);
        backLeftWheel.updateWithController(0, 0);
        backRightWheel.updateWithController(0, 0);
        telemetrySender.putSystemMessage("drive wheels time (ms)", System.currentTimeMillis() - t0);

        t0 = System.currentTimeMillis();
        for (ProfiledServo servo:servos)
            servo.update((System.currentTimeMillis() - previousTimeMillis) / 1000.0f);
        previousTimeMillis = System.currentTimeMillis();
        telemetrySender.putSystemMessage("servos update time (ms)", System.currentTimeMillis() - t0);
    }

    public void updateSensorsForever() {
        while (programRunningStatusChecker.isProgramActive())
            flushSensorsAndControllers();
    }

    public void updateMotorsForever() {
        while (programRunningStatusChecker.isProgramActive())
            flushMotorsAndServos();
    }

    public void stopRobot() {
        for (RobotModule robotModule:robotModules)
            robotModule.terminate();
    }

    private void initModulesAndService() {
        telemetrySender.init();
        for (RobotModule robotModule:robotModules) {
            robotModule.init();
            telemetrySender.addRobotModule(robotModule);
        }
        for (RobotService robotService:robotServices) {
            robotService.init();
            telemetrySender.addRobotService(robotService);
        }
    }

    private void configureChassisMotors() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void configureChassisWheels() {
        frontLeftWheel.setEncoderReversed(this.hardwareConfigs.frontLeftWheel_encoderReversed);
        frontRightWheel.setEncoderReversed(this.hardwareConfigs.frontRightWheel_encoderReversed);
        backLeftWheel.setEncoderReversed(this.hardwareConfigs.backLeftWheel_encoderReversed);
        backRightWheel.setEncoderReversed(this.hardwareConfigs.backRightWheel_encoderReversed);

        frontLeftWheel.setMotorReversed(this.hardwareConfigs.frontLeftWheel_motorReversed);
        frontRightWheel.setMotorReversed(this.hardwareConfigs.frontRightWheel_motorReversed);
        backLeftWheel.setMotorReversed(this.hardwareConfigs.backLeftWheel_motorReversed);
        backRightWheel.setMotorReversed(this.hardwareConfigs.backRightWheel_motorReversed);
    }
}
