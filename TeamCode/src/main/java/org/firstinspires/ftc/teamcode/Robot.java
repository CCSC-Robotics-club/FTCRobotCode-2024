package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Arm;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Modules.FixedAnglePixelCamera;
import org.firstinspires.ftc.teamcode.Modules.FlippableDualClaw;
import org.firstinspires.ftc.teamcode.Modules.TripleIndependentEncoderAndIMUPositionEstimator;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.DriverGamePad;
import org.firstinspires.ftc.teamcode.Utils.HuskyAprilTagCamera;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.EncoderMotorMechanism;
import org.firstinspires.ftc.teamcode.Utils.PositionEstimator;
import org.firstinspires.ftc.teamcode.Utils.ProgramRunningStatusChecker;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

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

    public FixedAnglePixelCamera pixelCamera;
    public PositionEstimator positionEstimator;
    public FixedAngleArilTagCamera aprilTagCamera;
    protected DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    protected final List<RobotModule> robotModules = new ArrayList<>(1);
    protected final List<RobotService> robotServices = new ArrayList<>(1);
    public TelemetrySender telemetrySender;
    protected IMU imu, alternativeIMU;

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

        // this.useMultiThread = !debugModeEnabled;
        this.useMultiThread = false;

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
        this.positionEstimator = new TripleIndependentEncoderAndIMUPositionEstimator(
                hardwareMap.get(DcMotor.class, encoderNames[0]),
                hardwareMap.get(DcMotor.class, encoderNames[1]),
                hardwareMap.get(DcMotor.class, encoderNames[2]),
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
        robotModules.add(aprilTagCamera);

//        pixelCamera = new FixedAnglePixelCamera(
//                new TensorCamera(hardwareMap.get(WebcamName.class, "Webcam 1")),
//                RobotConfig.VisualNavigationConfigs.pixelCameraSetUpProfile,
//                RobotConfig.VisualNavigationConfigs.pixelCameraInstallFacing
//        );
//        robotModules.add(pixelCamera);
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
        arm = new Arm(hardwareMap.get(DcMotor.class, "arm"), hardwareMap.get(DcMotor.class, "arm"), hardwareMap.get(TouchSensor.class, "armLimit"));
        robotModules.add(arm);
        /* claw */
        claw = new FlippableDualClaw(hardwareMap.get(Servo.class, "flip"), hardwareMap.get(Servo.class, "clawLeft"), hardwareMap.get(Servo.class, "clawRight"), arm);
        robotModules.add(claw);
    }

    /**
     * initializes the robot
     */
    public void initializeRobot() {
        /* <-- start of program --> */
        initModulesAndService();

        if (useMultiThread)
            scheduleThreads();

        hardwareMap.get(DcMotor.class, "servoModule").setPower(1);
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

    Thread updateServicesThread;
    List<Thread> updateModulesThreads = new ArrayList<>(1);
    private Thread chassisThread;
    private void scheduleThreads() {
        updateServicesThread = new Thread(new Runnable() {
            private double previousTimeMillis = System.currentTimeMillis();
            @Override
            public void run() {
                while (programRunningStatusChecker.isProgramActive()) {
                    if (driverGamePad != null) driverGamePad.update();
                    for (RobotService robotService : robotServices)
                        robotService.periodic();
                    telemetrySender.periodic();

                    while (System.currentTimeMillis() - previousTimeMillis + 3 < 1000f / RobotConfig.ControlConfigs.pilotControllerKeyUpdatingRate) {
                        try {
                            Thread.sleep(3);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    }

                    telemetrySender.putSystemMessage("services update rate", 1000 / (System.currentTimeMillis() - previousTimeMillis));
                    telemetrySender.putSystemMessage("chassis thread status", chassisThread.getState());
                    previousTimeMillis = System.currentTimeMillis();
                }
            }
        });

        for (RobotModule module : robotModules) {
            Runnable moduleUpdateRunnable = module.getRunnable(programRunningStatusChecker);
            updateModulesThreads.add(new Thread(moduleUpdateRunnable));
            if (module == chassis)
                chassisThread = updateModulesThreads.get(updateModulesThreads.size()-1);
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
