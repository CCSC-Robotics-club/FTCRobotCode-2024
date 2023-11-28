package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public abstract class AutoStageProgram {
    public final List<SequentialCommandSegment> commandSegments = new ArrayList<>(1);
    public final Robot.Side allianceSide;

    protected AutoStageProgram(Robot.Side side) {
        allianceSide = side;
    }

    public abstract void scheduleCommands(Chassis chassis, DistanceSensor distanceSensor, FixedAngleArilTagCamera angleArilTagCamera, ModulesCommanderMarker commanderMarker, TelemetrySender telemetrySender);
}
