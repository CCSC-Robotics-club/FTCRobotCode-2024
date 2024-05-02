package org.firstinspires.ftc.teamcode.Utils.HardwareUtils;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

public class ThreadedSensor {
    private final DoubleSupplier sensorReader;
    private double sensorReading;
    protected final Lock lock;

    public ThreadedSensor(DoubleSupplier sensorReader) {
        this(sensorReader, 0);
    }

    public ThreadedSensor(DoubleSupplier sensorReader, double startingReading) {
        this.sensorReader = sensorReader;
        this.lock = new ReentrantLock();
        this.sensorReading = startingReading;
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