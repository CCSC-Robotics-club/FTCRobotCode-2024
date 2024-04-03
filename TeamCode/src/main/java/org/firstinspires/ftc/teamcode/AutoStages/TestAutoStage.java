package org.firstinspires.ftc.teamcode.AutoStages;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Modules.Arm;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Modules.FixedAnglePixelCamera;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;

public class TestAutoStage extends AutoStageProgram {
    public TestAutoStage(Robot.Side side) {
        super(side);
    }

    @Override
    public void scheduleCommands(Chassis chassis, DistanceSensor distanceSensor, FixedAngleArilTagCamera angleArilTagCamera, Arm arm, Intake intake, FixedAnglePixelCamera pixelCamera, ModulesCommanderMarker commanderMarker, TelemetrySender telemetrySender) {
        super.commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(new Vector2D(new double[] {0, 0}), new Vector2D(new double[] {0, -50})),
                ()->{},
                ()->{},
                ()->{},
                chassis::isCurrentTranslationalTaskRoughlyComplete,
                () -> new Rotation2D(0), () -> new Rotation2D(0)
        ));
    }
}
