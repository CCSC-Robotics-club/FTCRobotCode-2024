package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.firstinspires.ftc.teamcode.AutoStages.BackFieldAutoTwoPieces;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "<Auto> [Red Back Stage Two Pieces]")
public class RedBackFieldTwoPiecesAuto extends AutoMain {
    public RedBackFieldTwoPiecesAuto() {
        super(new BackFieldAutoTwoPieces(Robot.Side.RED));
    }
}