package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Modules.ArmLegacy;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Modules.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Modules.FixedAnglePixelCamera;
import org.firstinspires.ftc.teamcode.Modules.IntakeLegacy;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;

import java.util.ArrayList;
import java.util.List;

public abstract class AutoStageProgram {
    public final List<SequentialCommandSegment> commandSegments = new ArrayList<>(1);
    public final Robot.Side allianceSide;

    protected AutoStageProgram(Robot.Side side) {
        allianceSide = side;
    }

    public abstract void scheduleCommands(HardwareMap hardwareMap, Chassis chassis, PositionEstimator positionEstimator, DistanceSensor distanceSensor, FixedAngleArilTagCamera angleArilTagCamera, ArmLegacy arm, IntakeLegacy intake, FixedAnglePixelCamera pixelCamera, ModulesCommanderMarker commanderMarker, TelemetrySender telemetrySender);
}
