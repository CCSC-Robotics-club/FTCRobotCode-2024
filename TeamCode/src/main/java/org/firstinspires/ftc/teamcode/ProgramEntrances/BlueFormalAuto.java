package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.firstinspires.ftc.teamcode.AutoStages.FormalAutoStage;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "<Auto> [Blue Auto Front Stage]")
public class BlueFormalAuto extends AutoMain {
    public BlueFormalAuto() {
        super(new FormalAutoStage(Robot.Side.BLUE));
    }
}
