package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.TeleOpMain;

@TeleOp(name = "<Debug-Mode> [Red Alliance]")
public class DebugModeRemoteControl extends TeleOpMain {
    public DebugModeRemoteControl() {
        super(Robot.Side.RED, true);
    }
}
