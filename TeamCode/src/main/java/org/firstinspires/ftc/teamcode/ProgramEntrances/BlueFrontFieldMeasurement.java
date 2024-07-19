package org.firstinspires.ftc.teamcode.ProgramEntrances;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.firstinspires.ftc.teamcode.AutoStages.OdometerMeasurement;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "<Measurement> [Blue] Front-Side Field Measurement")
public class BlueFrontFieldMeasurement extends AutoMain {
    public BlueFrontFieldMeasurement() {
        super(new OdometerMeasurement(Robot.Side.BLUE, true));
    }
}
