package org.firstinspires.ftc.teamcode.AutoStages;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Modules.Arm;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Modules.FixedAnglePixelCamera;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.PositionEstimator;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandFactory;

public class TestAutoStage extends AutoStageProgram {
    public TestAutoStage(Robot.Side side) {
        super(side);
    }

    @Override
    public void scheduleCommands(HardwareMap hardwareMap, Chassis chassis, PositionEstimator positionEstimator, DistanceSensor distanceSensor, FixedAngleArilTagCamera angleArilTagCamera, Arm arm, Intake intake, FixedAnglePixelCamera pixelCamera, ModulesCommanderMarker commanderMarker, TelemetrySender telemetrySender) {
        final SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(chassis, positionEstimator, "split first", new Rotation2D(Math.toRadians(90)), super.allianceSide, hardwareMap);
        super.commandSegments.add(sequentialCommandFactory.calibratePositionEstimator());
        super.commandSegments.addAll(
                sequentialCommandFactory.followPathFacing("split first", new Rotation2D(0))
        );
    }
}
