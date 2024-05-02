package org.firstinspires.ftc.teamcode.Utils.HardwareUtils;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.AngleUtils;


public class ThreadedIMU extends ThreadedSensor {
    private final IMU imu;
    private double previousYaw, yawVelocity;
    private long previousTimeMillis;
    public ThreadedIMU(IMU imu) {
        super(() -> imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        this.imu = imu;
        this.previousYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        this.yawVelocity = 0;
        this.previousTimeMillis = System.currentTimeMillis();
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    @Override
    public void update() {
        super.update();
        final double yaw = super.getSensorReading();
        super.lock.lock();
        yawVelocity = AngleUtils.getActualDifference(previousYaw, yaw) / (System.currentTimeMillis() - previousTimeMillis) * 1000.0;
        super.lock.unlock();
        previousTimeMillis = System.currentTimeMillis();
        previousYaw = yaw;
    }

    public double getYawVelocity() {
        super.lock.lock();
        final double yawVelocity = this.yawVelocity;
        super.lock.unlock();
        return yawVelocity;
    }
}
