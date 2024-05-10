package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FlippableDualClaw;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.PixelStackGrabbingCommand;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;

public class AutoGrabbingTestWithPixelStackLocator extends AutoStageProgram {
    public AutoGrabbingTestWithPixelStackLocator(Robot.Side side) {
        super(side);
    }

    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        // distance from wall to first stack: (18 + 131.5, 20)
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, "test path", new Rotation2D(0), super.allianceSide, robot.hardwareMap);
        /*
         * the position at which the center of the claw is above the stack
         * */
        final Vector2D stackCenterPositionDefault = new Vector2D(new double[] {18 + 135, 20});

        super.commandSegments.add(sequentialCommandFactory.calibratePositionEstimator());
        super.commandSegments.add(sequentialCommandFactory.justDoIt(() -> {
                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, null);
                    robot.claw.setLeftClawClosed(true, null);
                    robot.claw.setRightClawClosed(true, null);
        }));
        super.commandSegments.addAll(sequentialCommandFactory.followPathFacing("test path", new Rotation2D(0)));
        super.commandSegments.addAll(PixelStackGrabbingCommand.getCommandSegmentSegmentsWithColorSensor(robot, robot.spikeMarkDetectionSensor, sequentialCommandFactory, stackCenterPositionDefault));
        super.commandSegments.addAll(sequentialCommandFactory.followPathFacing("test path 2", new Rotation2D(0)));

    }
}
