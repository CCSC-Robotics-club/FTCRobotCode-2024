package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FlippableDualClaw;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;

public class OdometerMeasurement extends AutoStageProgram {
    private final boolean frontStage;
    public OdometerMeasurement(Robot.Side side, boolean frontStage) {
        super(side);
        this.frontStage = frontStage;
    }

    @Override
    public void beforeStartPeriodic() {}

    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(
                robot.chassis, robot.positionEstimator,
                frontStage ? "split left front stage" : "split left back stage",
                frontStage ? new Rotation2D(0) : new Rotation2D(Math.toRadians(180)),
                super.allianceSide,
                robot.hardwareMap);
        super.commandSegments.add(sequentialCommandFactory.calibratePositionEstimator());
        super.commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> {},
                () -> {
                    robot.claw.setRightClawClosed(true, null);
                    robot.claw.setLeftClawClosed(true, null);
                    robot.claw.setFlip(FlippableDualClaw.FlipperPosition.INTAKE, null);
                    robot.extend.setExtendPosition(RobotConfig.ExtendConfigs.intakeValue, null);
                    robot.arm.setPosition(RobotConfig.ArmConfigs.Position.INTAKE, null);
                    robot.chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY, new Vector2D()), null);
                },
                () -> {},
                () -> false,
                () -> new Rotation2D(0), () -> new Rotation2D(0)
        ));
    }
}
