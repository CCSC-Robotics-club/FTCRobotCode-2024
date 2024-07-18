package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.firstinspires.ftc.teamcode.AutoStages.TestTeamPropFinder;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "<Auto> [Test Team Finder]")
public class TestAutoStageProgram extends AutoMain {
    public TestAutoStageProgram() {
        super(new TestTeamPropFinder(Robot.Side.RED));
    }
}
