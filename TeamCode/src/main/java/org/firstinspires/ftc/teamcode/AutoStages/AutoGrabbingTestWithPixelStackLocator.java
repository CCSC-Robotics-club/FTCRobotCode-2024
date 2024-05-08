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
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(robot.chassis, robot.positionEstimator, super.allianceSide, robot.hardwareMap);
        /*
         * the position at which the center of the claw is above the stack
         * */
        final Vector2D stackCenterPositionDefault = new Vector2D(new double[] {0, -50});

        super.commandSegments.addAll(PixelStackGrabbingCommand.getCommandSegmentSegments(robot, sequentialCommandFactory, stackCenterPositionDefault));
    }
}
