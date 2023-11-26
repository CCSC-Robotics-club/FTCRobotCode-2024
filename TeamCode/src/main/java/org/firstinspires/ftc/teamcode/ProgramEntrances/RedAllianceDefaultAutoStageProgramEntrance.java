package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoStageRobot;
import org.firstinspires.ftc.teamcode.AutoStages.RedAllianceAutoStageProgramDefault;
import org.firstinspires.ftc.teamcode.RobotConfig;

@Autonomous(name = "[Red Alliance] Auto-Default")
public class RedAllianceDefaultAutoStageProgramEntrance extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutoStageRobot robot = new AutoStageRobot(
                hardwareMap,
                telemetry,
                () -> opModeIsActive() && !isStopRequested(),
                RobotConfig.hardwareConfigs_2024Competition,
                true,
                new RedAllianceAutoStageProgramDefault());

        robot.initializeRobot();
        waitForStart();
        robot.startRobot();
    }
}
