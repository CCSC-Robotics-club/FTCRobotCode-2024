package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

import static org.firstinspires.ftc.teamcode.RobotConfig.ArmConfigs;

import java.util.HashMap;
import java.util.Map;

public class Arm extends RobotModule {
    private final Servo armServo1, armServo2;

    private ArmConfigs.Position desiredPosition;
    private double servoCurrentPosition;
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
        final double servoDemandedPosition = ArmConfigs.servoPositions.get(desiredPosition);
        if (Math.abs(servoCurrentPosition - servoDemandedPosition) < dt * ArmConfigs.servoSpeed)
            servoCurrentPosition = servoDemandedPosition;
        else
            servoCurrentPosition += dt * ArmConfigs.servoSpeed * (servoCurrentPosition < servoDemandedPosition ? 1:-1);

        armServo1.setPosition(servoCurrentPosition);
        armServo2.setPosition(servoCurrentPosition);
    }

    @Override
    protected void onDestroy() {
        
    }

    @Override
    public void reset() {
        this.desiredPosition = ArmConfigs.Position.INTAKE;
        servoCurrentPosition = ArmConfigs.servoPositions.get(ArmConfigs.Position.INTAKE);
    }

    public void setPosition(ArmConfigs.Position position, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        this.desiredPosition = position;
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        final Map<String, Object> debugMessages = new HashMap<>();
        debugMessages.put("arm current position" , desiredPosition);
        debugMessages.put("arm position servo value", ArmConfigs.servoPositions.get(desiredPosition));
        return debugMessages;
    }
}
