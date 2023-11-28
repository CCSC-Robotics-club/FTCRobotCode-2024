package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Services.AutoProgramRunner;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.ProgramRunningStatusChecker;

public class AutoStageRobot extends Robot {
    private final AutoStageProgram autoStageProgram;
    final AutoProgramRunner autoProgramRunnerService;
    public AutoStageRobot(HardwareMap hardwareMap, Telemetry telemetry, ProgramRunningStatusChecker checker, RobotConfig.HardwareConfigs hardwareConfigs, AutoStageProgram autoStageProgram) {
        super(hardwareMap, telemetry, checker, hardwareConfigs, true, autoStageProgram.allianceSide, true);
        if (hardwareConfigs.encodersParams == null || hardwareConfigs.encoderNames == null)
            throw new IllegalStateException("auto stage cannot proceed without encoders");
        autoStageProgram.scheduleCommands(chassis, hardwareMap.get(DistanceSensor.class, "distance"));
        this.autoStageProgram = autoStageProgram;

        this.autoProgramRunnerService = new AutoProgramRunner(autoStageProgram.commandSegments, super.chassis);
        robotServices.add(autoProgramRunnerService);
    }

    @Override
    public void updateRobot() {
        super.updateRobot();
        if (autoProgramRunnerService.isAutoStageComplete())
            super.stopRobot();
    }

    public Chassis getChassisModule() {
        return super.chassis;
    }

    public Intake getIntakeModule() {
        return super.intake;
    }

    public FixedAngleArilTagCamera getAprilTagCamera() {
        return super.aprilTagCamera;
    }

    public HardwareMap getHardwareMap() {
        return super.hardwareMap;
    }

    public Telemetry getTelemetry() {
        return super.telemetry;
    }
}