package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoStageRobot;
import org.firstinspires.ftc.teamcode.AutoStages.AutoStageColorRecognitionBased;
import org.firstinspires.ftc.teamcode.AutoStages.AutoStageDistanceSensorBased;
import org.firstinspires.ftc.teamcode.RobotConfig;

@Autonomous(name="[Red Alliance] Front-Field Auto")
public class RedAllianceFrontFieldAutoStageProgramEntrance extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutoStageRobot robot = new AutoStageRobot(
                hardwareMap,
                telemetry,
                () -> opModeIsActive() && !isStopRequested(),
                RobotConfig.hardwareConfigs_2024Competition,
                new AutoStageColorRecognitionBased(AutoStageDistanceSensorBased.AutoStageConstantsTables.redAllianceFrontField));

        robot.initializeRobot();

        waitForStart();

        robot.startRobot();

        while (robot.programRunningStatusChecker.isProgramActive())
            robot.updateRobot();
    }
}
