package org.firstinspires.ftc.teamcode.AutoStages;


import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.AprilTagCameraAndDistanceSensorAimBot;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;
import org.firstinspires.ftc.teamcode.Utils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
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

    public void scheduleCommands(Chassis chassis, DistanceSensor distanceSensor, FixedAngleArilTagCamera aprilTagCamera, ModulesCommanderMarker commanderMarker) {
        AprilTagCameraAndDistanceSensorAimBot aimBot = new AprilTagCameraAndDistanceSensorAimBot(chassis, distanceSensor, aprilTagCamera, commanderMarker);
        BezierCurve path = new BezierCurve(
                new Vector2D(new double[] {0, 0}),
                new Vector2D(new double[] {25, 5}),
                new Vector2D(new double[] {15, 20}),
                new Vector2D(new double[] {15, 100})
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
                new Vector2D(new double[] {15, 100}),
                new Vector2D(new double[] {15, 110}),
                new Vector2D(new double[] {50, 200}),
                new Vector2D(new double[] {75, 215})
        );
        commandSegments.add(new SequentialCommandSegment(
                path,
                () -> {},
                () -> {},
                () -> {},
                () -> true,
                0, 0
        ));

        commandSegments.add(aimBot.createCommandSegment(new Vector2D(new double[] {0, -6})));
    }
}
