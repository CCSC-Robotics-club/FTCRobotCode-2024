package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.firstinspires.ftc.teamcode.AutoStages.FrontFieldAutoFourPieces;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "<Auto> [Blue Front Stage Two Pieces]")
public class BlueFrontTwoPiecesAuto extends AutoMain {
    public BlueFrontTwoPiecesAuto() {
        super(new FrontFieldAutoFourPieces(Robot.Side.BLUE));
    }
}
