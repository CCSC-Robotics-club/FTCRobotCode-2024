package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public abstract class AutoStageProgram {
    public final List<SequentialCommandSegment> commandSegments = new ArrayList<>(1);
    public final Robot.Side allianceSide;

    protected AutoStageProgram(Robot.Side side) {
        allianceSide = side;
    }

    public abstract void scheduleCommands(Robot robot, TelemetrySender telemetrySender);

    /**
     * called periodically after schedule commands, but before the start of the competition
     * to reduce CPU stress, this method is only called 20 times a second
     * */
    public abstract void beforeStartPeriodic();
    public void waitForStart(BooleanSupplier isStopRequested, BooleanSupplier isStarted) {
        while (!isStopRequested.getAsBoolean() && !isStarted.getAsBoolean()) {
            beforeStartPeriodic();
            try {
                Thread.sleep(50);
            } catch (InterruptedException ignored) {}
        }
    }
}
