package org.firstinspires.ftc.teamcode.Utils.HardwareUtils;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.MathUtils.AngleUtils;

import java.util.function.DoubleSupplier;

public class ThreadedEncoder extends SimpleSensor {
    private double previousReading, velocity;
    private long previousTimeMillis;
    public ThreadedEncoder(DcMotor encoder) {
        super(encoder::getCurrentPosition, encoder.getCurrentPosition());
        this.previousReading = encoder.getCurrentPosition();
        this.velocity = 0;
        this.previousTimeMillis = System.currentTimeMillis();
    }

    @Override
    public void update() {
        super.update();
        final double reading = super.getSensorReading();
        super.lock.lock();
        velocity = (reading - previousReading) / (System.currentTimeMillis() - previousTimeMillis) * 1000.0;
        System.out.print("encoder current reading: " + reading + ", ");
        System.out.print("previous reading: " + previousReading + ", ");
        System.out.print("time elapsed: " + ((System.currentTimeMillis() - previousTimeMillis) / 1000.0) + ", ");
        System.out.println("calculated velocity: " + velocity);
        super.lock.unlock();
        previousTimeMillis = System.currentTimeMillis();
        previousReading = reading;
    }

    public double getVelocity() {
        super.lock.lock();
        final double velocity = this.velocity;
        super.lock.unlock();
        return velocity;
    }
}
