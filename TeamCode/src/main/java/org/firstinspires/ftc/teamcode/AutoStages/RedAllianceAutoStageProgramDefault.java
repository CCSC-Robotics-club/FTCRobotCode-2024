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
                new Vector2D(new double[] {5, 5}),
                new Vector2D(new double[] {10, 40}),
                new Vector2D(new double[] {10, 60})
        );
        commandSegments.add(new SequentialCommandSegment(
                path,
                () -> {
                    telemetry.update();
                    telemetry.addLine("program started");
                    telemetry.update();
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                },
                () -> {},
                () -> {
                    telemetry.update();
                    telemetry.addLine("program ended");
                    telemetry.update();
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                },
                () -> true,
                0, Math.PI / 2r
        ));
    }
}
