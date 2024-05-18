package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.TeamElementFinder;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;

public class TestTeamPropRec extends AutoStageProgram {
    public TestTeamPropRec(Robot.Side side) {
        super(side);
    }

    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        final TeamElementFinder teamElementFinder = new TeamElementFinder(robot.hardwareMap, super.allianceSide);

        super.commandSegments.add(teamElementFinder.findTeamElementAndShutDown(5000));
        super.commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> telemetrySender.putSystemMessage("team prop pos", teamElementFinder.teamElementPosition), () -> {}, () -> {},
                () -> false,
                () -> null, () -> null
        ));
    }
}
