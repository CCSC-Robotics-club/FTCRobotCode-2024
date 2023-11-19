package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TeleOpMain extends LinearOpMode {
    private final Robot.Side side;
    public TeleOpMain(Robot.Side side) {
        this.side = side;
    }

    @Override
    public void runOpMode() {
        Robot robot = new Robot(
                hardwareMap,
                telemetry,
                () -> opModeIsActive() && !isStopRequested(),
                RobotConfig.hardwareConfigs_newer_chassis,
                gamepad1,
                gamepad2,
                true,
                this.side,
                false
        );

        robot.initializeRobot();

        waitForStart();
        robot.startRobot();

        while (opModeIsActive() && !isStopRequested()) robot.updateRobot();

        robot.stopRobot();
    }
}
