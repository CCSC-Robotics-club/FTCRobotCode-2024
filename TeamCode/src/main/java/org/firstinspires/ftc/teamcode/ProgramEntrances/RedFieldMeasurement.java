package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.firstinspires.ftc.teamcode.AutoStages.OdometerMeasurement;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "<Auto> [Red Field Measurement]")
public class RedFieldMeasurement extends AutoMain {
    public RedFieldMeasurement() {
        super(new OdometerMeasurement(Robot.Side.RED));
    }
}
