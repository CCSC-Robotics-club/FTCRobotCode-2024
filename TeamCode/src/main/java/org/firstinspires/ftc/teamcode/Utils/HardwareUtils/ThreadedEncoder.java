package org.firstinspires.ftc.teamcode.Utils.HardwareUtils;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.MathUtils.AngleUtils;

import java.util.function.DoubleSupplier;

public class ThreadedEncoder extends SimpleSensor {
    private final DcMotor encoder;
    private double previousReading, velocity;
    private long previousTimeMillis;
    public ThreadedEncoder(DcMotor encoder) {
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
        velocity = (reading - previousReading) / (System.currentTimeMillis() - previousTimeMillis);
        super.lock.unlock();
        previousTimeMillis = System.currentTimeMillis();
        previousReading = reading;
    }

    public double getVelocity() {
        return this.velocity;
    }
}
