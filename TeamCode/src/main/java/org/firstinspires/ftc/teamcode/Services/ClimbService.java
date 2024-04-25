package org.firstinspires.ftc.teamcode.Services;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Climb;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

public class ClimbService extends RobotService {
    private final Climb climb;
    private final Gamepad copilotGamepad;

    public ClimbService(Climb climb, Gamepad copilotGamepad) {
        this.climb = climb;
        this.copilotGamepad = copilotGamepad;
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic(double dt) {
        if (copilotGamepad.dpad_up)
            this.climb.startClimb();
    }

    @Override
    public void onDestroy() {

    }

    @Override
    public void reset() {

    }
}
