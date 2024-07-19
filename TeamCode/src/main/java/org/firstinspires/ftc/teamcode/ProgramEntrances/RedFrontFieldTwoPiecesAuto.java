package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.firstinspires.ftc.teamcode.AutoStages.FrontFieldAutoTwoPieces;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "<Auto> [Red Front Stage Two Pieces]")
public class RedFrontFieldTwoPiecesAuto extends AutoMain {
    public RedFrontFieldTwoPiecesAuto() {
        super(new FrontFieldAutoTwoPieces(Robot.Side.RED));
    }
}
