package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class MotorThreaded {
    private final DcMotor motor;
    private final Lock lock;
    private double power = 0;

    public MotorThreaded(DcMotor motor) {
        this.motor = motor;
        this.lock = new ReentrantLock();
    }

    public void update() {
        lock.lock();
        final double power = this.power;
        lock.unlock();
        this.motor.setPower(power);
    }

    public void setPower(double power) {
        lock.lock();
        this.power = power;
        lock.unlock();
    }

    public DcMotor getMotorInstance() {
        return this.motor;
    }
}
