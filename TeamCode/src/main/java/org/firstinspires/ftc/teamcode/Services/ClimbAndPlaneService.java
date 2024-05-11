package org.firstinspires.ftc.teamcode.Services;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.ClimbAndPlane;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

public class ClimbAndPlaneService extends RobotService {
    private final ClimbAndPlane climbAndPlane;
    private final Gamepad copilotGamepad;

    public ClimbAndPlaneService(ClimbAndPlane climbAndPlane, Gamepad copilotGamepad) {
        this.climbAndPlane = climbAndPlane;
        this.copilotGamepad = copilotGamepad;
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic(double dt) {
        if (copilotGamepad.dpad_up)
            this.climbAndPlane.startClimb();
    }

    @Override
    public void onDestroy() {

    }

    @Override
    public void reset() {

    }
}
