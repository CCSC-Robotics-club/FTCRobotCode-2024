package org.firstinspires.ftc.teamcode.Utils.HardwareUtils;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

public class ThreadedSensor {
    private final DoubleSupplier sensorReader;
    private double sensorReading;
    private final double defaultReading;
    private boolean enabled;
    protected final Lock lock;

    public ThreadedSensor(DoubleSupplier sensorReader) {
        this(sensorReader, 0);
    }

    public ThreadedSensor(DoubleSupplier sensorReader, double defaultReading) {
        this.sensorReader = sensorReader;
        this.lock = new ReentrantLock();
        this.defaultReading = this.sensorReading = defaultReading;

        enabled = true;
    }

    public void update() {
        if (!enabled)
            return;
        final double newValue = sensorReader.getAsDouble();
        lock.lock();
        this.sensorReading = newValue;
        lock.unlock();
    }

    public double getSensorReading() {
        if (!enabled) return defaultReading;
        lock.lock();
        final double sensorReading = this.sensorReading;
        lock.unlock();
        return sensorReading;
    }

    public void setEnabled(boolean enabled) {
        if (!this.enabled && enabled) {
            this.enabled = true;update();
        }

        this.enabled = enabled;
    }
}