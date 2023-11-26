package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Services.AutoProgramRunner;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.ProgramRunningStatusChecker;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;

public class AutoStageRobot extends Robot {
    private final AutoStageProgram autoStageProgram;
    public AutoStageRobot(HardwareMap hardwareMap, Telemetry telemetry, ProgramRunningStatusChecker checker, RobotConfig.HardwareConfigs hardwareConfigs, boolean visualNavigationSupported, AutoStageProgram autoStageProgram) {
        super(hardwareMap, telemetry, checker, hardwareConfigs, visualNavigationSupported, autoStageProgram.allianceSide, true);
        this.autoStageProgram = autoStageProgram;
    }

    @Override
    public void initializeRobot() {
        super.initializeRobot();

        final AutoProgramRunner autoProgramRunnerService = new AutoProgramRunner(autoStageProgram.commandSegments, super.chassis, super.telemetry);

        robotServices.add(autoProgramRunnerService);
    }
}