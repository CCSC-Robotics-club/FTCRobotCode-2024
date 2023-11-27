package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TeleOpMain extends LinearOpMode {
    private final Robot.Side side;
    private final boolean enableDebugMode;
    public TeleOpMain(Robot.Side side, boolean enableDebugMode) {
        this.side = side;
        this.enableDebugMode = enableDebugMode;
    }

    @Override
    public void runOpMode() {
        Robot robot = new ManualStageRobot(
                hardwareMap,
                telemetry,
                () -> opModeIsActive() && !isStopRequested(),
                RobotConfig.hardwareConfigs_newer_chassis,
                gamepad1,
                gamepad2,
                false,
                this.side,
                enableDebugMode
        );

        robot.initializeRobot();

        waitForStart();
        robot.startRobot();

        while (opModeIsActive() && !isStopRequested()) robot.updateRobot();

        robot.stopRobot();
    }
}
