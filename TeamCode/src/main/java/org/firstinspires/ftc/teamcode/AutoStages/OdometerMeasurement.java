package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Services.TelemetrySender;
import org.firstinspires.ftc.teamcode.Utils.AutoStageProgram;

public class OdometerMeasurement extends AutoStageProgram {
    protected OdometerMeasurement(Robot.Side side) {
        super(side);
    }

    @Override
    public void scheduleCommands(Robot robot, TelemetrySender telemetrySender) {
        // TODO use odometer to measure positions on field
    }
}
