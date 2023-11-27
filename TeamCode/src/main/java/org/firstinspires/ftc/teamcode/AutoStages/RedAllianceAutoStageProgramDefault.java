package org.firstinspires.ftc.teamcode.AutoStages;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.Vector2D;

import java.util.ArrayList;
import java.util.List;

public final class RedAllianceAutoStageProgramDefault extends AutoStageProgram {
    public RedAllianceAutoStageProgramDefault(Telemetry telemetry) {
        super(Robot.Side.RED);

        BezierCurve path = new BezierCurve(
                new Vector2D(new double[] {0, 0}),
                new Vector2D(new double[] {10, 5}),
                new Vector2D(new double[] {10, 20}),
                new Vector2D(new double[] {10, 60})
        );
        commandSegments.add(new SequentialCommandSegment(
                path,
                () -> {},
                () -> {},
                () -> {},
                () -> true,
                0, 0
        ));

        path = new BezierCurve(
                new Vector2D(new double[] {10, 60}),
                new Vector2D(new double[] {10, 90}),
                new Vector2D(new double[] {50, 190}),
                new Vector2D(new double[] {70, 190})
        );
        commandSegments.add(new SequentialCommandSegment(
                path,
                () -> {},
                () -> {},
                () -> {},
                () -> true,
                0, 0
        ));
    }
}
