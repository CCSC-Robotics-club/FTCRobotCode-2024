package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Services.ClimbAndPlaneService;
import org.firstinspires.ftc.teamcode.Services.PilotChassisService;
import org.firstinspires.ftc.teamcode.Services.UpperStructureService;
import org.firstinspires.ftc.teamcode.Utils.DriverGamePad;
import org.firstinspires.ftc.teamcode.Utils.ProgramRunningStatusChecker;


public class ManualStageRobot extends Robot {
    private final Gamepad copilotGamepad;
    private final Side side;
    public ManualStageRobot(HardwareMap hardwareMap, Telemetry telemetry, ProgramRunningStatusChecker checker, RobotConfig.HardwareConfigs hardwareConfigs, Gamepad gamepad1, Gamepad gamepad2, Side side, boolean debugModeEnabled) {
        super(hardwareMap, telemetry, checker, hardwareConfigs, side, debugModeEnabled);
        super.driverGamePad = new DriverGamePad(gamepad1);
        this.copilotGamepad = gamepad2;
        this.side = side;
    }

    @Override
    public void initializeRobot() {
        final double pilotFacing = side == Side.RED ? Math.PI / 2 : -Math.PI / 2;
        PilotChassisService chassisService = new PilotChassisService(chassis, driverGamePad, copilotGamepad, super.distanceSensor, pixelCamera, pilotFacing);
        super.robotServices.add(chassisService);

//        final IntakeService intakeService = new IntakeService(intake, chassisService.pixelDetector, copilotGamepad);
//        super.robotServices.add(intakeService);
//
//        final ArmService armService = new ArmService(super.arm, copilotGamepad);
//        super.robotServices.add(armService);
//
//        final PlaneLauncher planeLauncher = new PlaneLauncher(hardwareMap.get(Servo.class, "launch"), hardwareMap.get(Servo.class, "lift"));
//        super.robotModules.add(planeLauncher);
//        final PlaneLaunchService planeLaunchService = new PlaneLaunchService(planeLauncher, copilotGamepad);
//        super.robotServices.add(planeLaunchService);

        final UpperStructureService upperStructureService = new UpperStructureService(arm, extend, claw, chassisService, copilotGamepad);
        super.robotServices.add(upperStructureService);

        final ClimbAndPlaneService climbAndPlaneService = new ClimbAndPlaneService(climbAndPlane, copilotGamepad);
        super.robotServices.add(climbAndPlaneService);

        super.initializeRobot();

        super.distanceSensorBack.setEnabled(false);
        super.spikeMarkDetectionSensor.setEnabled(false);
    }
}
