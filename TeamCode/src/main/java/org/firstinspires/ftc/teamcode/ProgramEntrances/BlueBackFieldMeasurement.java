package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.firstinspires.ftc.teamcode.AutoStages.OdometerMeasurement;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "<Auto> [Measurement] Back-Side Field Measurement")
public class BlueBackFieldMeasurement extends AutoMain {
    public BlueBackFieldMeasurement() {
        super(new OdometerMeasurement(Robot.Side.BLUE, false));
    }
}