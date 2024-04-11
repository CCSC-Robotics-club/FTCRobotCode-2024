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

        if (limitSwitch.isPressed())
            this.armEncoderZeroPosition = armEncoder.getCurrentPosition();
        else if (armEncoderZeroPosition == -114514) {
            this.armMotor.setPower(-ArmConfigs.powerNeededToMoveDown * motorPowerFactor);
            return;
        }

        armController.desiredPosition = ArmConfigs.encoderPositions.get(desiredPosition);
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
    }

    public void setPosition(ArmConfigs.Position position, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
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

    @Override
    public Map<String, Object> getDebugMessages() {
        final Map<String, Object> debugMessages = new HashMap<>();

        debugMessages.put("arm desired position (enc)", ArmConfigs.encoderPositions.get(desiredPosition));
        debugMessages.put("arm current position (enc)" , getArmEncoderPosition());
        debugMessages.put("arm in position", isArmInPosition());
        return debugMessages;
    }
}
