package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.List;

public abstract class AutoStageProgram {
    public final List<SequentialCommandSegment> commandSegments = new ArrayList<>(1);
    public final Robot.Side allianceSide;

    protected AutoStageProgram(Robot.Side side) {
        allianceSide = side;
    }
}
