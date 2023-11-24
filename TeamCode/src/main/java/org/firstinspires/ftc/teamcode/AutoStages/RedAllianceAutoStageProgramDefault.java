package org.firstinspires.ftc.teamcode.AutoStages;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.Vector2D;

import java.util.ArrayList;
import java.util.List;

public final class RedAllianceAutoStageProgramDefault extends AutoStageProgram {
    public RedAllianceAutoStageProgramDefault() {
        super(Robot.Side.RED);

        List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        long t0 = System.currentTimeMillis();
        BezierCurve path = new BezierCurve(
                new Vector2D(new double[] {0, 0}),
                new Vector2D(new double[] {0, 50}),
                new Vector2D(new double[] {150, 100}),
                new Vector2D(new double[] {200, 100})
        );
        commandSegments.add(new SequentialCommandSegment(
                path,
                () -> {},
                () -> {},
                () -> {},
                () -> true,
                0, Math.PI / 2
        ));
    }
}
