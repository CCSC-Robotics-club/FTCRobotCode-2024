package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoStageRobot;
import org.firstinspires.ftc.teamcode.AutoStages.BlueAllianceAutoStageProgramDefault;
import org.firstinspires.ftc.teamcode.RobotConfig;

@Autonomous(name = "[Blue Alliance] Auto-Default")
public class BlueAllianceDefaultAutoStageProgramEntrance extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoStageRobot robot = new AutoStageRobot(
                hardwareMap,
                telemetry,
                () -> opModeIsActive() && !isStopRequested(),
                RobotConfig.hardwareConfigs_newer_chassis,
                new BlueAllianceAutoStageProgramDefault(telemetry));

        robot.initializeRobot();

        waitForStart();

        robot.startRobot();

        while (opModeIsActive() && !isStopRequested())
            robot.updateRobot();
    }
}
