package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

import static org.firstinspires.ftc.teamcode.RobotConfig.ArmConfigs;

public class Arm extends RobotModule {
    private final Servo armServo1, armServo2;

    private ArmConfigs.Position position;
    public Arm(Servo armServo1, Servo armServo2) {
        super("arm");
        this.armServo1 = armServo1;
        this.armServo2 = armServo2;
    }

    @Override
    public void init() {
        armServo1.setDirection(ArmConfigs.servo1Reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        armServo2.setDirection(ArmConfigs.servo2Reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        reset();
    }

    @Override
    protected void periodic(double dt) {
        armServo1.setPosition(ArmConfigs.encoderPositions.get(position));
        armServo2.setPosition(ArmConfigs.encoderPositions.get(position));
    }

    @Override
    protected void onDestroy() {

    }

    @Override
    public void reset() {
        this.position = ArmConfigs.Position.INTAKE;
    }

    public void setPosition(ArmConfigs.Position position, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        this.position = position;
    }
}
