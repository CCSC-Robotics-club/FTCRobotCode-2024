package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.firstinspires.ftc.teamcode.AutoStages.BackFieldAutoTwoPieces;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "<Auto> [Blue Back Stage Two Pieces]")
public class BlueBackFieldTwoPiecesAuto extends AutoMain {
    public BlueBackFieldTwoPiecesAuto() {
        super(new BackFieldAutoTwoPieces(Robot.Side.BLUE));
    }
}
