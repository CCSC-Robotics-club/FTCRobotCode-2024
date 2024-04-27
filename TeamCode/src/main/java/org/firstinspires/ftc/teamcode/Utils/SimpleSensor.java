package org.firstinspires.ftc.teamcode.Utils;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

public class SimpleSensor {
    private final DoubleSupplier sensorReader;
    private double sensorReading = Double.NaN;
    private final Lock lock;

    public SimpleSensor(DoubleSupplier sensorReader) {
        this.sensorReader = sensorReader;
        this.lock = new ReentrantLock();
    }

    public void update() {
        final double newValue = sensorReader.getAsDouble();
        lock.lock();
        this.sensorReading = newValue;
        lock.unlock();
    }

    public double getSensorReading() {
        lock.lock();
        final double sensorReading = this.sensorReading;
        lock.unlock();
        return sensorReading;
    }
}