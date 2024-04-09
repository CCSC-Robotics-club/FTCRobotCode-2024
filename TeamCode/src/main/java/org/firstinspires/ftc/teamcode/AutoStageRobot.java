package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Modules.IntakeLegacy;
import org.firstinspires.ftc.teamcode.Services.AutoProgramRunner;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.ProgramRunningStatusChecker;

public class AutoStageRobot extends Robot {
    private final AutoStageProgram autoStageProgram;
    final AutoProgramRunner autoProgramRunnerService;
    public AutoStageRobot(HardwareMap hardwareMap, Telemetry telemetry, ProgramRunningStatusChecker checker, RobotConfig.HardwareConfigs hardwareConfigs, AutoStageProgram autoStageProgram) {
        super(hardwareMap, telemetry, null, hardwareConfigs, autoStageProgram.allianceSide, false);
        if (hardwareConfigs.encodersParams == null || hardwareConfigs.encoderNames == null)
            throw new IllegalStateException("auto stage cannot proceed without encoders");

        this.autoStageProgram = autoStageProgram;
        this.autoProgramRunnerService = new AutoProgramRunner(super.chassis);
        super.programRunningStatusChecker = () -> checker.isProgramActive() && (!autoProgramRunnerService.isAutoStageComplete());
    }

    @Override
    public void initializeRobot() {
        autoStageProgram.scheduleCommands(this, autoProgramRunnerService, telemetrySender);

        robotServices.add(autoProgramRunnerService);
        super.initializeRobot();

        autoProgramRunnerService.scheduleCommandSegments(autoStageProgram.commandSegments);
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