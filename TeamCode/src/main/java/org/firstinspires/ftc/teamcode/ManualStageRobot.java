package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Services.IntakeService;
import org.firstinspires.ftc.teamcode.Services.PilotChassisService;
import org.firstinspires.ftc.teamcode.Utils.DriverGamePad;
import org.firstinspires.ftc.teamcode.Utils.ProgramRunningStatusChecker;


public class ManualStageRobot extends Robot {
    private final Gamepad copilotGamepad;
    public ManualStageRobot(HardwareMap hardwareMap, Telemetry telemetry, ProgramRunningStatusChecker checker, RobotConfig.HardwareConfigs hardwareConfigs, Gamepad gamepad1, Gamepad gamepad2, boolean visualNavigationSupported, Side side, boolean debugModeEnabled) {
        super(hardwareMap, telemetry, checker, hardwareConfigs, visualNavigationSupported, side, debugModeEnabled);
        super.driverGamePad = new DriverGamePad(gamepad1);
        this.copilotGamepad = gamepad2;
    }

    @Override
    public void initializeRobot() {
        super.initializeRobot();
        PilotChassisService chassisService = new PilotChassisService(chassis, driverGamePad, hardwareMap.get(DistanceSensor.class, "distance"), independentEncodersAvailable, visualNavigationSupported);
        super.robotServices.add(chassisService);

        final IntakeService intakeService = new IntakeService(intake, driverGamePad, copilotGamepad);
        super.robotServices.add(intakeService);
    }
}
