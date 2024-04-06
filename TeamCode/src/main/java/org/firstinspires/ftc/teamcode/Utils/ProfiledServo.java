package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class ProfiledServo {
    private final Servo servo;
    private final double servoSpeed;

    private double desiredPosition, servoCurrentPosition;
    public ProfiledServo(Servo servo, double servoSpeed) {
        this.servo = servo;
        this.servoSpeed = servoSpeed;

        desiredPosition = 0;
        servoCurrentPosition = 0;
    }

    public void setDesiredPosition(double desiredPosition) {
        this.desiredPosition = desiredPosition;
    }

    public void update(double dt) {
        if (Math.abs(servoCurrentPosition - desiredPosition) < dt * servoSpeed)
            servoCurrentPosition = desiredPosition;
        else
            servoCurrentPosition += dt * servoSpeed * (servoCurrentPosition < desiredPosition ? 1:-1);

        // servo.setPosition(servoCurrentPosition);
        servo.setPosition(desiredPosition);
    }

    public boolean inPosition() {
        return servoCurrentPosition == desiredPosition;
    }
}
