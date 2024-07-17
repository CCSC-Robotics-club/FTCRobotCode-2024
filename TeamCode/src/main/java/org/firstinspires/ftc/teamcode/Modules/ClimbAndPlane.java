package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.RobotModule;

public class ClimbAndPlane extends RobotModule {
    private long climbTaskStart = 0;
    private final Servo climb0, climb1, plane;
    private final DcMotor climbMotor0, climbMotor1;
    private double climbPower;
    public ClimbAndPlane(Servo climb0, Servo climb1, Servo plane, DcMotor climbMotor0, DcMotor climbMotor1) {
        super("climb");
        this.climb0 = climb0;
        this.climb1 = climb1;
        this.plane = plane;
        this.climbMotor0 = climbMotor0;
        this.climbMotor1 = climbMotor1;
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    protected void periodic(double dt) {
        boolean servoOnTrigger = System.currentTimeMillis() - climbTaskStart < 500;
        climb0.setPosition(servoOnTrigger ? 0: 0.5);
        climb1.setPosition(servoOnTrigger ? 1: 0.5);
        plane.setPosition(servoOnTrigger ? 0 : 0.5);

        climbMotor0.setPower(climbPower);
        climbMotor1.setPower(-climbPower);
    }

    @Override
    protected void onDestroy() {

    }

    @Override
    public void reset() {
        climb0.setPosition(0.5);
        climb1.setPosition(0.5);
        plane.setPosition(0.5);
        climbPower = 0;
    }

    public void startClimb() {
        this.climbTaskStart = System.currentTimeMillis();
    }

    public void setClimbPower(double power) {
        this.climbPower = power;
    }
}
