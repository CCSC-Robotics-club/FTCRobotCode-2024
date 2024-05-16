package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.TeamElementFinder;

public class TestAutoRoute extends AutoStageProgram {
    public TestAutoRoute(Robot.Side side) {
        super(side);
    }

    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, "split first(left)", new Rotation2D(Math.toRadians(0)), super.allianceSide, robot.hardwareMap);
        super.commandSegments.add(sequentialCommandFactory.calibratePositionEstimator());
        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("split first(left)", new Rotation2D(0))
        );

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("move back and grab third from stack", new Rotation2D(0))
        );

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("grab fourth from stack", new Rotation2D(0))
        );

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("score third and fourth", new Rotation2D(0))
        );

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("move to stack grab fifth", new Rotation2D(0))
        );

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("grab sixth from stack", new Rotation2D(0))
        );

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("score fifth and sixth", new Rotation2D(0))
        );

        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("park", new Rotation2D(0))
        );
    }
}
