package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;

public class TestAutoStage extends AutoStageProgram {
    public TestAutoStage(Robot.Side side) {
        super(side);
    }

    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, super.allianceSide, robot.hardwareMap);
        super.commandSegments.add(sequentialCommandFactory.calibratePositionEstimator());
        super.commandSegments.add(sequentialCommandFactory.moveToPoint(
                new Vector2D(new double[] {0, 50})
        ));
    }
}
