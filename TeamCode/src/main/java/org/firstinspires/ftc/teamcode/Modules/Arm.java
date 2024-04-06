package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.ProfiledServo;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

import static org.firstinspires.ftc.teamcode.RobotConfig.ArmConfigs;

import java.util.HashMap;
import java.util.Map;

public class Arm extends RobotModule {
    private final ProfiledServo armServo1, armServo2;

    private ArmConfigs.Position desiredPosition;
    private double servoCurrentPosition;
    public Arm(Servo armServo1, Servo armServo2) {
        super("arm");
        armServo1.setDirection(ArmConfigs.servo1Reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        armServo2.setDirection(ArmConfigs.servo2Reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        this.armServo1 = new ProfiledServo(armServo1, ArmConfigs.servoSpeed);
        this.armServo2 = new ProfiledServo(armServo2, ArmConfigs.servoSpeed);
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    protected void periodic(double dt) {
        final double servoDemandedPosition = ArmConfigs.servoPositions.get(desiredPosition);
        armServo1.setDesiredPosition(servoDemandedPosition);
        armServo2.setDesiredPosition(servoDemandedPosition);

        armServo1.update(dt);
        armServo2.update(dt);
    }

    @Override
    protected void onDestroy() {
        
    }

    @Override
    public void reset() {
        this.desiredPosition = ArmConfigs.Position.INTAKE;
        armServo1.setDesiredPosition(ArmConfigs.servoPositions.get(desiredPosition));
        armServo2.setDesiredPosition(ArmConfigs.servoPositions.get(desiredPosition));
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
