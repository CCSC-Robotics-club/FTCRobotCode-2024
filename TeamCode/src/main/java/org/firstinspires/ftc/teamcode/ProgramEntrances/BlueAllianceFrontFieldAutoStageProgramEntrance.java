package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoStageRobot;
import org.firstinspires.ftc.teamcode.AutoStages.AutoStageDefault;
import org.firstinspires.ftc.teamcode.RobotConfig;

@Autonomous(name = "[Blue Alliance] Front-Field Auto")
public class BlueAllianceFrontFieldAutoStageProgramEntrance extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoStageRobot robot = new AutoStageRobot(
                hardwareMap,
                telemetry,
                () -> opModeIsActive() && !isStopRequested(),
                RobotConfig.hardwareConfigs_2024Competition,
                new AutoStageDefault(AutoStageDefault.AutoStageConstantsTables.blueAllianceFrontField, true));

        robot.initializeRobot();

        waitForStart();

        robot.startRobot();

        while (robot.programRunningStatusChecker.isProgramActive())
            robot.updateRobot();
    }
}
