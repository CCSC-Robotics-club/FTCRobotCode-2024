package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoStageRobot;
import org.firstinspires.ftc.teamcode.AutoStages.AutoStageColorRecognitionBased;
import org.firstinspires.ftc.teamcode.AutoStages.AutoStageDistanceSensorBased;
import org.firstinspires.ftc.teamcode.RobotConfig;

@Autonomous(name = "[Blue Alliance] Front-Field Auto")
public class BlueAllianceFrontFieldAutoStageProgramEntrance extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoStageRobot robot = new AutoStageRobot(
                hardwareMap,
                telemetry,
                () -> opModeIsActive() && !isStopRequested(),
                RobotConfig.competitionConfig,
                new AutoStageColorRecognitionBased(AutoStageColorRecognitionBased.AutoStageConstantsTables.blueAllianceFrontField));

        robot.initializeRobot();

        waitForStart();

        robot.startRobot();

        while (robot.programRunningStatusChecker.isProgramActive())
            robot.updateRobot();
    }
}
