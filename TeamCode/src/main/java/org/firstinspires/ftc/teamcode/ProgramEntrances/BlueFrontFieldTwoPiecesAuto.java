package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.firstinspires.ftc.teamcode.AutoStages.FrontFieldAutoTwoPieces;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "<Auto> [Blue Front Stage Two Pieces]")
public class BlueFrontFieldTwoPiecesAuto extends AutoMain {
    public BlueFrontFieldTwoPiecesAuto() {
        super(new FrontFieldAutoTwoPieces(Robot.Side.BLUE));
    }
}
