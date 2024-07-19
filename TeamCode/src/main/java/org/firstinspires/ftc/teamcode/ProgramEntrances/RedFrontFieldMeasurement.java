package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.firstinspires.ftc.teamcode.AutoStages.OdometerMeasurement;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "<Measurement> [Red] Front-Side Field Measurement")
public class RedFrontFieldMeasurement extends AutoMain {
    public RedFrontFieldMeasurement() {
        super(new OdometerMeasurement(Robot.Side.RED, true));
    }
}
