package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;

public class AutoMain extends LinearOpMode {
    final AutoStageProgram autoStageProgram;

    public AutoMain(AutoStageProgram autoStageProgram) {
        this.autoStageProgram = autoStageProgram;
    }

    @Override
    public void runOpMode() {
        AutoStageRobot robot = new AutoStageRobot(
                gamepad1,gamepad2,
                hardwareMap,
                telemetry,
                () -> opModeIsActive() && !isStopRequested(),
                RobotConfig.competitionConfig,
                autoStageProgram
        );

        robot.initializeRobot();

        autoStageProgram.waitForStart(this::isStopRequested, this::isStarted);

        robot.startRobot();

        while (robot.programRunningStatusChecker.isProgramActive())
            robot.updateRobot();
    }
}