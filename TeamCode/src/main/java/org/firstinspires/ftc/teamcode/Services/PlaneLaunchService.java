package org.firstinspires.ftc.teamcode.Services;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.PlaneLauncher;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

public class PlaneLaunchService extends RobotService {
    private final PlaneLauncher planeLauncher;
    private final Gamepad copilotGamePad;
    public PlaneLaunchService(PlaneLauncher planeLauncher, Gamepad copilotGamePad) {
        this.planeLauncher = planeLauncher;
        this.copilotGamePad = copilotGamePad;
    }
    @Override
    public void init() {
        reset();
    }

    @Override
    public void periodic(double dt) {
        if (copilotGamePad.dpad_down)
            this.planeLauncher.launchPlane(this);
    }

    @Override
    public void onDestroy() {

    }

    @Override
    public void reset() {
        planeLauncher.gainOwnerShip(this);
    }
}
