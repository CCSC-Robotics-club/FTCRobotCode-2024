package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.Arm;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.Climb;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.FixedAnglePixelCamera;
import org.firstinspires.ftc.teamcode.Modules.FlippableDualClaw;
import org.firstinspires.ftc.teamcode.Modules.TripleIndependentEncoderAndIMUPositionEstimator;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.DriverGamePad;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.HuskyAprilTagCamera;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.EncoderMotorMechanism;
import org.firstinspires.ftc.teamcode.Utils.MotorThreaded;
import org.firstinspires.ftc.teamcode.Utils.PositionEstimator;
import org.firstinspires.ftc.teamcode.Utils.ProfiledServo;
import org.firstinspires.ftc.teamcode.Utils.ProgramRunningStatusChecker;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.RobotService;
import org.firstinspires.ftc.teamcode.Utils.SimpleSensor;

import java.util.ArrayList;
import java.util.List;

public abstract class Robot {
    public final HardwareMap hardwareMap;
    protected final RobotConfig.HardwareConfigs hardwareConfigs;
    public final Telemetry telemetry;
    public ProgramRunningStatusChecker programRunningStatusChecker;
    protected DriverGamePad driverGamePad = null;

    protected final boolean useMultiThread;

    private final EncoderMotorMechanism frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel;
    public DistanceSensor distanceSensor, distanceSensorBack;
    public Chassis chassis;
//    public IntakeLegacy intake;
//    public ArmLegacy arm;

    public Arm arm;
    public FlippableDualClaw claw;

    public Climb climb;

    public FixedAnglePixelCamera pixelCamera;
    public PositionEstimator positionEstimator;
    public FixedAngleArilTagCamera aprilTagCamera;
    protected DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    protected final List<RobotModule> robotModules = new ArrayList<>(1);
    protected final List<RobotService> robotServices = new ArrayList<>(1);
    public TelemetrySender telemetrySender;
    protected IMU imu, alternativeIMU;
    private final List<SimpleSensor> sensors = new ArrayList<>();
    private final List<MotorThreaded> motors = new ArrayList<>();
    private final List<ProfiledServo> servos = new ArrayList<>();

    public enum Side {
        RED,
        BLUE
    }
    protected final Side side;
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, ProgramRunningStatusChecker checker, RobotConfig.HardwareConfigs hardwareConfigs, Side side) {
        this(hardwareMap, telemetry, checker, hardwareConfigs, side, false);
    }
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, ProgramRunningStatusChecker checker, RobotConfig.HardwareConfigs hardwareConfigs, Side side, boolean debugModeEnabled) {
        this.side = side;

        this.hardwareConfigs = hardwareConfigs;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.programRunningStatusChecker = checker;

        this.useMultiThread = !debugModeEnabled;
        // this.useMultiThread = false;

        telemetrySender = new TelemetrySender(telemetry);

        /* <-- chassis --> */
        this.frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        this.frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        this.backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        this.backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");

        this.imu = hardwareMap.get(IMU.class, "imu");
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        this.distanceSensorBack = hardwareMap.get(DistanceSensor.class, "distanceBack");
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

        final SimpleSensor horizontalEncoder = new SimpleSensor(() -> hardwareMap.get(DcMotor.class, encoderNames[0]).getCurrentPosition()),
                verticalEncoder1 = new SimpleSensor(() -> hardwareMap.get(DcMotor.class, encoderNames[1]).getCurrentPosition()),
                verticalEncoder2 = new SimpleSensor(() -> hardwareMap.get(DcMotor.class, encoderNames[2]).getCurrentPosition()),
                imuSensor = new SimpleSensor(() -> imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        sensors.add(horizontalEncoder);
        sensors.add(verticalEncoder1);
        sensors.add(verticalEncoder2);
        sensors.add(imuSensor);


        this.positionEstimator = new TripleIndependentEncoderAndIMUPositionEstimator(
                horizontalEncoder,
                verticalEncoder1,
                verticalEncoder2,
                imuSensor,
                imu,
                this.hardwareConfigs.encodersParams
        );
        ((RobotModule) positionEstimator).init();
        robotModules.add((RobotModule) positionEstimator);

        configureChassisWheels();

        aprilTagCamera = new FixedAngleArilTagCamera(
                new HuskyAprilTagCamera(hardwareMap.get(HuskyLens.class, "husky")),
                RobotConfig.VisualNavigationConfigs.visualCameraProfile
        );

        pixelCamera = null;

        chassis = new Chassis(frontLeftWheel, frontRightWheel, backLeftWheel ,backRightWheel, positionEstimator, aprilTagCamera,
                this.side == Side.RED ? FixedAngleArilTagCamera.WallTarget.Name.RED_ALLIANCE_WALL : FixedAngleArilTagCamera.WallTarget.Name.BLUE_ALLIANCE_WALL);
        robotModules.add(chassis);
//
//
//        /* <-- intake --> */
//        final DcMotor intakeMotor1 = hardwareMap.get(DcMotor.class, RobotConfig.IntakeConfigs.intakeMotor1Name),
//                intakeMotor2 = hardwareMap.get(DcMotor.class, RobotConfig.IntakeConfigs.intakeMotor2Name);
//        intake = new Intake(intakeMotor1, intakeMotor2);
//        robotModules.add(intake);
//
//        /* <-- arm --> */
//        SingleServoClaw claw1 = new SingleServoClaw(hardwareMap.get(Servo.class, RobotConfig.ArmConfigs.claw1Name), RobotConfig.ArmConfigs.claw1Profile);
//        Claw claw;
//        if (RobotConfig.ArmConfigs.claw2Name == null) {
//            claw = claw1;
//        } else {
//            SingleServoClaw claw2 = new SingleServoClaw(hardwareMap.get(Servo.class, RobotConfig.ArmConfigs.claw2Name), RobotConfig.ArmConfigs.claw2Profile);
//            claw = new DualServoClaw(claw1, claw2);
//        }
//        final DcMotor armMotor1 = hardwareMap.get(DcMotor.class, RobotConfig.ArmConfigs.armMotor1Name),
//                armMotor2 = hardwareMap.get(DcMotor.class, RobotConfig.ArmConfigs.armMotor2Name),
//                armEncoder = hardwareMap.get(DcMotor.class, RobotConfig.ArmConfigs.armEncoderName);
//        final TouchSensor limitSwitch = RobotConfig.ArmConfigs.limitSwitchName != null ? hardwareMap.get(TouchSensor.class, RobotConfig.ArmConfigs.limitSwitchName) : null;
//        final ExtendableClaw extendableClaw = new ExtendableClaw(claw, hardwareMap.get(Servo.class, "extend"));
//        arm = new Arm(armMotor1, armMotor2, armEncoder, extendableClaw, limitSwitch);
//        robotModules.add(arm);
//        robotModules.add(extendableClaw);



        /* arm */
        final MotorThreaded armMotor = new MotorThreaded(hardwareMap.get(DcMotor.class, "arm"));
        final SimpleSensor armEncoder = new SimpleSensor(() -> hardwareMap.get(DcMotor.class, "arm").getCurrentPosition()),
                armLimit = new SimpleSensor(() -> hardwareMap.get(TouchSensor.class, "armLimit").isPressed() ? 1:0);
        motors.add(armMotor);
        sensors.add(armEncoder);
        sensors.add(armLimit);
        arm = new Arm(armMotor, armEncoder, armLimit);
        robotModules.add(arm);


        /* claw */
        final ProfiledServo flip = new ProfiledServo(hardwareMap.get(Servo.class, "flip"), 2),
                clawLeft = new ProfiledServo(hardwareMap.get(Servo.class, "clawLeft"), 2),
                clawRight = new ProfiledServo(hardwareMap.get(Servo.class, "clawRight"), 2);
        final MotorThreaded indicatorLightLeft = new MotorThreaded(hardwareMap.get(DcMotor.class, "indicatorLightLeft")),
                indicatorLightRight = new MotorThreaded(hardwareMap.get(DcMotor.class, "indicatorLightRight"));
        final SimpleSensor colorLeft = new SimpleSensor(() -> hardwareMap.get(ColorSensor.class, "colorLeft").alpha()),
                colorRight = new SimpleSensor(() -> hardwareMap.get(ColorSensor.class, "colorRight").alpha());
        servos.add(flip);
        servos.add(clawLeft);
        servos.add(clawRight);
        sensors.add(colorLeft);
        sensors.add(colorRight);
        motors.add(indicatorLightLeft);
        motors.add(indicatorLightRight);
        claw = new FlippableDualClaw(
                flip, clawLeft, clawRight,
                arm, colorLeft, colorRight,
                indicatorLightLeft, indicatorLightRight
        );
        robotModules.add(claw);

        /* climb */
        climb = new Climb(
                hardwareMap.get(Servo.class, "climb0"),
                hardwareMap.get(Servo.class, "climb1")
        );
        robotModules.add(climb);
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
        for (SimpleSensor sensor: sensors)
            sensor.update(); // TODO detailed timings
        telemetrySender.putSystemMessage("sensors update time(ms)", System.currentTimeMillis() - t0);
    }

    private void flushMotorsAndServos() {
        long t0 = System.currentTimeMillis();
        for (MotorThreaded motor:motors)
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
            servo.update(System.currentTimeMillis() - previousTimeMillis);
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
