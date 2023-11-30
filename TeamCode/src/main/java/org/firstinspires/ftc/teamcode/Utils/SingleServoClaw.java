package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.Servo;

public class SingleServoClaw implements Claw {
    private final double closeValue, openValue;
    private final Servo servo;
    public SingleServoClaw(Servo servo, ServoProfile servoProfile) {
        this.servo = servo;

        this.closeValue = servoProfile.closeValue;
        this.openValue = servoProfile.openValue;
    }
    @Override
    public void close() {
        servo.setPosition(closeValue);
    }

    @Override
    public void open() {
        servo.setPosition(openValue);
    }

    @Override
    public boolean isClosed() {
        return Math.abs(servo.getPosition() - closeValue) < Math.abs(servo.getPosition() - openValue);
    }
}
