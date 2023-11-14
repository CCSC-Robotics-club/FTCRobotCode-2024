package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.EncoderMotorWheel;
import org.firstinspires.ftc.teamcode.Modules.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Modules.TripleIndependentEncoderAndIMUPositionEstimator;
import org.firstinspires.ftc.teamcode.Services.PilotChassisService;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.DriverGamePad;
import org.firstinspires.ftc.teamcode.Utils.HuskyAprilTagCamera;
import org.firstinspires.ftc.teamcode.Utils.PositionEstimator;
import org.firstinspires.ftc.teamcode.Utils.ProgramRunningStatusChecker;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.RobotService;
import org.firstinspires.ftc.teamcode.Utils.SimpleFeedForwardSpeedController;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

public class Robot {
    private final HardwareMap hardwareMap;
    private final RobotConfig.HardwareConfigs hardwareConfigs;
    private final Telemetry telemetry;
    private final ProgramRunningStatusChecker programRunningStatusChecker;

    private final boolean visualNavigationSupported, independentEncodersAvailable, useMultiThread;
    private boolean robotStopped = false;

    private final Timer timer = new Timer();
    public final List<TimerTask> tasks = new ArrayList<>(1);
    private EncoderMotorWheel frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel;
    private DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private List<RobotModule> robotModules = new ArrayList<>(1);
    private List<RobotService> robotServices = new ArrayList<>(1);
    private TelemetrySender telemetrySender;
    private DriverGamePad driverGamePad;
    private Gamepad armOperatorPad;
    private IMU imu, alternativeIMU;

    public enum Side {
        RED,
        BLUE
    }
    private final Side side;
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, ProgramRunningStatusChecker checker, RobotConfig.HardwareConfigs hardwareConfigs, Gamepad gamepad1, Gamepad gamepad2, boolean visualNavigationSupported, Side side) {
        this(hardwareMap, telemetry, checker, hardwareConfigs, gamepad1, gamepad2, visualNavigationSupported, side, false);
    }
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, ProgramRunningStatusChecker checker, RobotConfig.HardwareConfigs hardwareConfigs, Gamepad gamepad1, Gamepad gamepad2, boolean visualNavigationSupported, Side side, boolean debugModeEnabled) {
        this.side = side;
        this.independentEncodersAvailable = hardwareConfigs.encodersParams != null && hardwareConfigs.encoderNames != null;

        this.hardwareConfigs = hardwareConfigs;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.programRunningStatusChecker = checker;
        this.visualNavigationSupported = visualNavigationSupported && independentEncodersAvailable; // visual would not be available without encoders

        this.useMultiThread = !debugModeEnabled;

        /* game pad */
        this.armOperatorPad = gamepad2;
        driverGamePad = new DriverGamePad(gamepad1);
    }

    /**
     * initializes the robot
     */
    public void initializeRobot() {
        /* hardware */
        this.frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        this.frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        this.backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        this.backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");

        this.imu = hardwareMap.get(IMU.class, "imu");
        if (hardwareConfigs.alternativeIMUParameter != null)
            this.alternativeIMU = hardwareMap.get(IMU.class, "alternativeIMU");
        else
            this.alternativeIMU = null;
        imu.initialize(this.hardwareConfigs.imuParameter);

        SimpleFeedForwardSpeedController.SpeedControllerProfile wheelSpeedControllerProfile =
                new SimpleFeedForwardSpeedController.SpeedControllerProfile(
                        RobotConfig.ChassisConfigs.wheel_proportionGain, RobotConfig.ChassisConfigs.wheel_feedForwardGain, RobotConfig.ChassisConfigs.wheel_feedForwardDelay);

        configureChassisMotors();
        frontLeftWheel = new EncoderMotorWheel(
                frontLeftMotor,
                frontLeftMotor,
                new SimpleFeedForwardSpeedController(wheelSpeedControllerProfile),
                RobotConfig.ChassisConfigs.wheel_maxVelocity
        );
        frontRightWheel = new EncoderMotorWheel(
                frontRightMotor,
                frontRightMotor,
                new SimpleFeedForwardSpeedController(wheelSpeedControllerProfile),
                RobotConfig.ChassisConfigs.wheel_maxVelocity
        );
        backLeftWheel = new EncoderMotorWheel(
                backLeftMotor,
                backLeftMotor,
                new SimpleFeedForwardSpeedController(wheelSpeedControllerProfile),
                RobotConfig.ChassisConfigs.wheel_maxVelocity
        );

        backRightWheel = new EncoderMotorWheel(
                backRightMotor,
                backRightMotor,
                new SimpleFeedForwardSpeedController(wheelSpeedControllerProfile),
                RobotConfig.ChassisConfigs.wheel_maxVelocity
        );

        String[] encoderNames = this.hardwareConfigs.encoderNames == null ?
                new String[] {"frontLeft", "frontRight", "backLeft"} :
                this.hardwareConfigs.encoderNames;
        PositionEstimator positionEstimator = new TripleIndependentEncoderAndIMUPositionEstimator(
                hardwareMap.get(DcMotor.class, encoderNames[0]),
                hardwareMap.get(DcMotor.class, encoderNames[1]),
                hardwareMap.get(DcMotor.class, encoderNames[2]),
                imu,
                alternativeIMU,
                this.hardwareConfigs.encodersParams
        );
        ((RobotModule) positionEstimator).init();
        robotModules.add((RobotModule) positionEstimator);

        configureChassisWheels();
        robotModules.add(frontLeftWheel);
        robotModules.add(frontRightWheel);
        robotModules.add(backLeftWheel);
        robotModules.add(backRightWheel);

        FixedAngleArilTagCamera aprilTagCamera;
        if (visualNavigationSupported) {
            aprilTagCamera = new FixedAngleArilTagCamera(
                    new HuskyAprilTagCamera(hardwareMap.get(HuskyLens.class, "husky")),
                    RobotConfig.VisualNavigationConfigs.visualCameraProfile
            );
            robotModules.add(aprilTagCamera);
        } else aprilTagCamera = null;



        Chassis chassis = new Chassis(frontLeftWheel, frontRightWheel, backLeftWheel ,backRightWheel, positionEstimator, aprilTagCamera, FixedAngleArilTagCamera.WallTarget.Name.RED_ALLIANCE_WALL);
        robotModules.add(chassis);

        PilotChassisService chassisService = new PilotChassisService(chassis, driverGamePad, hardwareMap.get(DistanceSensor.class, "distance"), independentEncodersAvailable, visualNavigationSupported);
        robotServices.add(chassisService);

        initModulesAndService();

        if (useMultiThread)
            scheduleThreads();

        telemetry.addLine("startup complete...");
        telemetry.update();
    }

    public void startRobot() {
        if (this.useMultiThread) runThreads();
    }

    public void updateRobot() {
        if (useMultiThread) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException ignored) {}
            return;
        }

        long t0 = System.currentTimeMillis();
        driverGamePad.update();
        telemetrySender.putSystemMessage("controller update time(ms)", System.currentTimeMillis() - t0);

        t0 = System.currentTimeMillis();
        for (RobotService robotService: robotServices)
            robotService.periodic();
        telemetrySender.putSystemMessage("robot service time(ms)", System.currentTimeMillis() - t0);

        for (RobotModule robotModule: robotModules) {
            t0 = System.currentTimeMillis();
            robotModule.periodic();
            telemetrySender.putSystemMessage(robotModule.toString(), System.currentTimeMillis() - t0);
        }

        telemetrySender.periodic();
    }

    public void stopRobot() {
        for (RobotModule robotModule:robotModules)
            robotModule.terminate();
        robotStopped = true;
    }

    private void initModulesAndService() {
        telemetrySender = new TelemetrySender(telemetry);
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

    Thread updateServicesThread;
    List<Thread> updateModulesThreads = new ArrayList<>(1);
    private void scheduleThreads() {
        updateServicesThread = new Thread(new Runnable() {
            private double previousTimeMillis = System.currentTimeMillis();
            @Override
            public void run() {
                while (programRunningStatusChecker.isProgramActive()) {
                    driverGamePad.update();
                    for (RobotService robotService : robotServices)
                        robotService.periodic((System.currentTimeMillis() - previousTimeMillis) / 1000.0f);
                    telemetrySender.periodic((System.currentTimeMillis() - previousTimeMillis) / 1000.0f);

                    while (System.currentTimeMillis() - previousTimeMillis + 3 < 1000d / RobotConfig.ControlConfigs.pilotControllerKeyUpdatingRate) {
                        try {
                            Thread.sleep(3);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    }

                    telemetrySender.putSystemMessage("services dt", System.currentTimeMillis() - previousTimeMillis);
                    previousTimeMillis = System.currentTimeMillis();
                }
            }
        });

        for (RobotModule module : robotModules) {
            Runnable moduleUpdateRunnable = module.getRunnable(programRunningStatusChecker);
            updateModulesThreads.add(new Thread(moduleUpdateRunnable));
        }
    }

    private void runThreads() {
        updateServicesThread.start();
        for (Thread moduleThread:updateModulesThreads)
            moduleThread.start();
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
