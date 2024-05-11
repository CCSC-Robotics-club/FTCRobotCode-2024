package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.RobotModule;

public class ClimbAndPlane extends RobotModule {
    private long climbTaskStart = 0;
    private final Servo climb0, climb1, plane;
    public ClimbAndPlane(Servo climb0, Servo climb1, Servo plane) {
        super("climb");
        this.climb0 = climb0;
        this.climb1 = climb1;
        this.plane = plane;
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    protected void periodic(double dt) {
        final double power = System.currentTimeMillis() - climbTaskStart < 500 ? 1 : 0.5;
        climb0.setPosition(power);
        climb1.setPosition(1-power);
        plane.setPosition(power);
    }

    @Override
    protected void onDestroy() {

    }

    @Override
    public void reset() {
        climb0.setPosition(0.5);
        climb1.setPosition(0.5);
        plane.setPosition(0.5);
    }

    public void startClimb() {
        this.climbTaskStart = System.currentTimeMillis();
    }
}
