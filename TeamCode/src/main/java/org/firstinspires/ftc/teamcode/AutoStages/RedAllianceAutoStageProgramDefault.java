package org.firstinspires.ftc.teamcode.AutoStages;


import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.Vector2D;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class RedAllianceAutoStageProgramDefault extends AutoStageProgram {
    private final Telemetry telemetry;
    public RedAllianceAutoStageProgramDefault(Telemetry telemetry) {
        super(Robot.Side.RED);
        this.telemetry = telemetry;
    }

    public void scheduleCommands(Chassis chassis, DistanceSensor distanceSensor) {
        BezierCurve path = new BezierCurve(
                new Vector2D(new double[] {0, 0}),
                new Vector2D(new double[] {25, 5}),
                new Vector2D(new double[] {15, 20}),
                new Vector2D(new double[] {15, 60})
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
                new Vector2D(new double[] {15, 60}),
                new Vector2D(new double[] {10, 90}),
                new Vector2D(new double[] {50, 200}),
                new Vector2D(new double[] {75, 210})
        );
        commandSegments.add(new SequentialCommandSegment(
                path,
                () -> {},
                () -> {},
                () -> {},
                () -> true,
                0, 0
        ));

        commandSegments.add(new SequentialCommandSegment(
                null,
                () -> {

                },
                () -> {},
                () -> {},
                () -> true,
                0, 0
        ));
    }
}
