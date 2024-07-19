package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.firstinspires.ftc.teamcode.AutoStages.OdometerMeasurement;
import org.firstinspires.ftc.teamcode.Robot;
@Autonomous(name = "<Measurement> [Red] Back-Side Field Measurement")

public class RedBackFieldMeasurement extends AutoMain {
    public RedBackFieldMeasurement() {
        super(new OdometerMeasurement(Robot.Side.RED, false));
    }
}