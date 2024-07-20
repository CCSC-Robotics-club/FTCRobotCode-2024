package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Services.AutoProgramRunner;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.ProgramRunningStatusChecker;

public class AutoStageRobot extends Robot {
    private final AutoStageProgram autoStageProgram;
    final AutoProgramRunner autoProgramRunnerService;
    public AutoStageRobot(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry, ProgramRunningStatusChecker checker, RobotConfig.HardwareConfigs hardwareConfigs, AutoStageProgram autoStageProgram) {
        super(gamepad1, gamepad2, hardwareMap, telemetry, null, hardwareConfigs, autoStageProgram.allianceSide, false);
        if (hardwareConfigs.encodersParams == null || hardwareConfigs.encoderNames == null)
            throw new IllegalStateException("auto stage cannot proceed without encoders");

        this.autoStageProgram = autoStageProgram;
        this.autoProgramRunnerService = new AutoProgramRunner(super.chassis);
        super.programRunningStatusChecker = () -> checker.isProgramActive() && (!autoProgramRunnerService.isAutoStageComplete());
    }

    @Override
    public void initializeRobot() {
        robotServices.add(autoProgramRunnerService);
        super.initializeRobot();

        autoStageProgram.scheduleCommands(this, telemetrySender);
        autoProgramRunnerService.setCommandSegments(autoStageProgram.commandSegments);
        super.colorLeft.setEnabled(false);
        super.colorRight.setEnabled(false);

//        super.spikeMarkDetectionSensor.setEnabled(false);
//        super.distanceSensorBack.setEnabled(false);
    }

    @Override
    public void updateRobot() {
        super.updateRobot();
        telemetrySender.putSystemMessage("chassis thread update count", chassis.updateCount);
        if (autoProgramRunnerService.isAutoStageComplete())
            super.stopRobot();
    }

    public HardwareMap getHardwareMap() {
        return super.hardwareMap;
    }

    public Telemetry getTelemetry() {
        return super.telemetry;
    }
}