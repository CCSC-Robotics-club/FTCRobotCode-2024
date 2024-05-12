package org.firstinspires.ftc.teamcode.ProgramEntrances;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.firstinspires.ftc.teamcode.AutoStages.OdometerMeasurement;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "<Auto> [Blue Field Measurement]")
public class FieldMeasurement extends AutoMain {
    public FieldMeasurement() {
        super(new OdometerMeasurement(Robot.Side.BLUE));
    }
}
