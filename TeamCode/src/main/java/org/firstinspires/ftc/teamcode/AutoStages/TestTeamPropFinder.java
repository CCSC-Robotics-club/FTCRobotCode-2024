package org.firstinspires.ftc.teamcode.AutoStages;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.TeamElementFinderColor;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;

public class TestTeamPropFinder extends AutoStageProgram {
    public TestTeamPropFinder(Robot.Side side) {
        super(side);
    }

    private TeamElementFinderColor teamElementFinderColor = null;
    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        teamElementFinderColor = new TeamElementFinderColor(
                robot.hardwareMap,
                robot.hardwareMap.get(WebcamName.class, "Webcam 1"),
                robot.gamepad1,
                robot.telemetry,
                super.allianceSide
        );
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, allianceSide, robot.hardwareMap);

        super.commandSegments.add(commandFactory.justDoIt(() -> {
            throw new RuntimeException("result: " + teamElementFinderColor.getBestResult());
        }));
    }

    @Override
    public void beforeStartPeriodic() {
        if (teamElementFinderColor != null)
            teamElementFinderColor.beforeStartPeriodic();
    }
}
