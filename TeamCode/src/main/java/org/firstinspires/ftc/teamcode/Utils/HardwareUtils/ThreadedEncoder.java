package org.firstinspires.ftc.teamcode.Utils.HardwareUtils;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ThreadedEncoder extends ThreadedSensor {
    private double previousReading, velocity;
    private long previousTimeMillis;
    private final DcMotorEx encoder;
    public ThreadedEncoder(DcMotorEx encoder) {
        super(encoder::getCurrentPosition, encoder.getCurrentPosition());
        this.previousReading = encoder.getCurrentPosition();
        this.velocity = 0;
        this.previousTimeMillis = System.currentTimeMillis();
        this.encoder = encoder;
    }

    @Override
    public void update() {
        super.update();
        final double reading = super.getSensorReading();
        super.lock.lock();
        velocity = (reading - previousReading) / (System.currentTimeMillis() - previousTimeMillis) * 1000.0;
        super.lock.unlock();
        previousTimeMillis = System.currentTimeMillis();
        previousReading = reading;

//        lock.lock();
//        this.velocity = encoder.getVelocity();
//        lock.unlock();
    }

    public double getVelocity() {
        super.lock.lock();
        final double velocity = this.velocity;
        super.lock.unlock();
        return velocity;
    }
}
