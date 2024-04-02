package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoStageRobot;
import org.firstinspires.ftc.teamcode.AutoStages.AutoStageDistanceSensorBased;
import org.firstinspires.ftc.teamcode.RobotConfig;

@Autonomous(name="[Blue Alliance] Back-Field Auto")
public class BlueAllianceBackFieldAutoStageProgramEntrance extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoStageRobot robot = new AutoStageRobot(
                hardwareMap,
                telemetry,
                () -> opModeIsActive() && !isStopRequested(),
                RobotConfig.competitionConfig,
                new AutoStageDistanceSensorBased(AutoStageDistanceSensorBased.AutoStageConstantsTables.blueAllianceBackField));

        robot.initializeRobot();

        waitForStart();

        robot.startRobot();

        while (robot.programRunningStatusChecker.isProgramActive())
            robot.updateRobot();
    }
}
