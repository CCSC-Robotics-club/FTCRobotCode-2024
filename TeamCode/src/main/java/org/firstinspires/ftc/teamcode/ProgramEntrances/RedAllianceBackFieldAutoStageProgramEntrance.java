package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoStageRobot;
import org.firstinspires.ftc.teamcode.AutoStages.AutoStageDistanceSensorBased;
import org.firstinspires.ftc.teamcode.RobotConfig;

// @Autonomous(name="[Red Alliance] Back-Field Auto")
public class RedAllianceBackFieldAutoStageProgramEntrance extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutoStageRobot robot = new AutoStageRobot(
                hardwareMap,
                telemetry,
                () -> opModeIsActive() && !isStopRequested(),
                RobotConfig.competitionConfig,
                new AutoStageDistanceSensorBased(AutoStageDistanceSensorBased.AutoStageConstantsTables.redAllianceBackField));

        robot.initializeRobot();

        waitForStart();

        robot.startRobot();

        while (robot.programRunningStatusChecker.isProgramActive())
            robot.updateRobot();
    }
}
