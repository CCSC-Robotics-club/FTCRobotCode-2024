package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.SimpleArmController;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

import static org.firstinspires.ftc.teamcode.RobotConfig.ArmConfigs;


import java.util.HashMap;
import java.util.Map;


public class Arm extends RobotModule {
    // TODO make the height of scoring change-able using lookup table, and update the visual navigation approaching distance as well
    private final DcMotor armMotor, armEncoder;
    private final TouchSensor limitSwitch;
    private int armEncoderZeroPosition = -114514;
    private double scoringHeight;
    private long intakeStatusStartTime;
    private boolean sensorPressed;

    private double desiredPower = 0;
    private final SimpleArmController armController = new SimpleArmController(
            ArmConfigs.maxPowerWhenMovingUp,
            ArmConfigs.maxPowerWhenMovingDown,
            ArmConfigs.errorStartDecelerate,
            ArmConfigs.powerNeededToMoveUp,
            ArmConfigs.powerNeededToMoveDown,
            ArmConfigs.errorTolerance,
            false
    );

    private ArmConfigs.Position desiredPosition;
    public Arm(DcMotor armMotor, DcMotor armEncoder, TouchSensor limitSwitch) {
        super("arm");
        this.armMotor = armMotor;
        this.armEncoder = armEncoder;
        this.limitSwitch = limitSwitch;
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    protected void periodic(double dt) {
        final double motorPowerFactor = ArmConfigs.motorReversed ? -1:1;

        if (sensorPressed = limitSwitch.isPressed())
            this.armEncoderZeroPosition = armEncoder.getCurrentPosition();

        if (armStuck() || Math.abs(desiredPower) > 0.05) {
            armMotor.setPower(motorPowerFactor * desiredPower);
            return;
        }
        if (armEncoderZeroPosition == -114514) {
            armMotor.setPower(-0.8 * motorPowerFactor);
            return;
        }

        armController.desiredPosition = ArmConfigs.encoderPositions.get(desiredPosition);
        if (desiredPosition == ArmConfigs.Position.INTAKE) {
            armMotor.setPower(limitSwitch.isPressed() ? 0 : motorPowerFactor * armController.getMotorPower(0, getArmEncoderPosition() + 100));
            return;
        }
        if (desiredPosition == ArmConfigs.Position.SCORE)
            armController.desiredPosition = ArmConfigs.armScoringAnglesAccordingToScoringHeight.getYPrediction(scoringHeight);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        final double currentPosition = getArmEncoderPosition(),
                correctionPower = armController.getMotorPower(0, currentPosition);
        if (currentPosition <= 0 && correctionPower < 0)
            armMotor.setPower(0);
        else
            armMotor.setPower(correctionPower * motorPowerFactor);
    }

    @Override
    protected void onDestroy() {
        
    }

    @Override
    public void reset() {
        this.desiredPosition = ArmConfigs.Position.INTAKE;
        this.scoringHeight = 1;

        intakeStatusStartTime = System.currentTimeMillis();
    }

    public void setPosition(ArmConfigs.Position position, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        if (this.desiredPosition != ArmConfigs.Position.INTAKE && position == ArmConfigs.Position.INTAKE)
            this.intakeStatusStartTime = System.currentTimeMillis();
        this.desiredPosition = position;
    }

    public boolean isArmInPosition() {
        return Math.abs(getArmEncoderPosition() - armController.desiredPosition) < ArmConfigs.errorTolerance * 2;
    }

    public int getArmEncoderPosition() {
        return (armEncoder.getCurrentPosition() - armEncoderZeroPosition) * (ArmConfigs.encoderReversed ? -1: 1);
    }

    public double getScoringOrHoldingClawAngle(double holdingPosition) {
        return desiredPosition == ArmConfigs.Position.SCORE
                ? ArmConfigs.flipperPositionsAccordingToScoringHeight.getYPrediction(scoringHeight)
                : holdingPosition;
    }

    public double getScoringDistanceToWall() {
        return -ArmConfigs.distancesToWallAccordingToScoringHeight.getYPrediction(scoringHeight);
    }

    public void setScoringHeight(double scoringHeight, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        this.scoringHeight = scoringHeight;
    }

    public boolean armStuck() {
        return !sensorPressed && System.currentTimeMillis() - intakeStatusStartTime > 5000 && this.desiredPosition == ArmConfigs.Position.INTAKE;
    }

    public void forceSetPower(double desiredPower, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        this.desiredPower = desiredPower;
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        final Map<String, Object> debugMessages = new HashMap<>();

        debugMessages.put("arm desired position (enc)", armController.desiredPosition);
        debugMessages.put("arm current position (enc)" , getArmEncoderPosition());
        debugMessages.put("arm in position", isArmInPosition());
        return debugMessages;
    }
}
