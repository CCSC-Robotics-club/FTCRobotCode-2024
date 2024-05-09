package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.firstinspires.ftc.teamcode.AutoStages.AutoGrabbingTestWithPixelStackLocator;
import org.firstinspires.ftc.teamcode.AutoStages.AutoStageGrabbingTest;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "<Auto> [Test Auto Stage]")
public class TestAutoStageProgram extends AutoMain {

    public TestAutoStageProgram() {
        super(new AutoGrabbingTestWithPixelStackLocator(Robot.Side.BLUE));
    }
}
