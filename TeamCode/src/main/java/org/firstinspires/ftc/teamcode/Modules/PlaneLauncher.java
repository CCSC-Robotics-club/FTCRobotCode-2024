package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;

import static org.firstinspires.ftc.teamcode.RobotConfig.LauncherConfigs;

public class PlaneLauncher extends RobotModule {
    private final Servo launcher, lift;
    private double timer = 0;

    public enum Status {
        STANDBY,
        LIFTING,
        LAUNCHING,
        FINISHED
    }

    private Status status;
    public PlaneLauncher(Servo launcher, Servo lift) {
        super("Plane-Launcher");
        this.launcher = launcher;
        this.lift = lift;
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    protected void periodic(double dt) {
        timer += dt;

        if (status == Status.LIFTING || status == Status.LAUNCHING)
            lift.setPosition(LauncherConfigs.liftActivatePosition);
        else
            lift.setPosition(LauncherConfigs.liftZeroPosition);
        if (status == Status.LAUNCHING || status == Status.FINISHED)
            launcher.setPosition(LauncherConfigs.launcherActivatePosition);
        else
            launcher.setPosition(LauncherConfigs.launcherZeroPosition);

        if (status == Status.LIFTING && timer > LauncherConfigs.servoMovementTime){
            status = Status.LAUNCHING;
            timer = 0;
        } else if (status == Status.LAUNCHING && timer > LauncherConfigs.servoMovementTime)
            status = Status.FINISHED;
    }

    @Override
    protected void onDestroy() {

    }

    @Override
    public void reset() {
        this.status = Status.STANDBY;
    }

    public void launchPlane(ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;
        if (this.status == Status.STANDBY) {
            this.status = Status.LIFTING;
            timer = 0;
        }
    }
}
