package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedEncoder;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedMotor;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.SimpleArmController;
import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.RobotService;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedSensor;

import static org.firstinspires.ftc.teamcode.RobotConfig.ArmConfigs;


import java.util.HashMap;
import java.util.Map;


public class Arm extends RobotModule {
    private final ThreadedMotor armMotor1, armMotor2;
    private final ThreadedEncoder armEncoder;
    private final ThreadedSensor limitSwitch;
    private int armEncoderZeroPosition = -114514;
    private double scoringHeight;
    private final SimpleArmController
            simpleArmControllerNormal = new SimpleArmController(
            ArmConfigs.maxPowerWhenMovingUpNormal,
            ArmConfigs.maxPowerWhenMovingDownNormal,
            ArmConfigs.errorStartDecelerateNormal,
            ArmConfigs.powerNeededToMoveUpNormal,
            ArmConfigs.powerNeededToMoveDownNormal,
            ArmConfigs.errorToleranceNormal,
            false
    ),
            simpleArmControllerScoring = new SimpleArmController(
            ArmConfigs.maxPowerWhenMovingUpScoring,
            ArmConfigs.maxPowerWhenMovingDownScoring,
            ArmConfigs.errorStartDecelerateScoring,
            ArmConfigs.powerNeededToMoveUpScoring,
            ArmConfigs.powerNeededToMoveDownScoring,
            ArmConfigs.errorToleranceScoring,
            false
    );

    private ArmConfigs.Position desiredPosition;
    public Arm(ThreadedMotor armMotor, ThreadedEncoder armEncoder, ThreadedSensor limitSwitch) {
        this(armMotor, armMotor, armEncoder, limitSwitch);
    }
    public Arm(ThreadedMotor armMotor1, ThreadedMotor armMotor2, ThreadedEncoder armEncoder, ThreadedSensor limitSwitch) {
        super("arm");
        this.armMotor1 = armMotor1;
        this.armMotor2 = armMotor2;
        this.armEncoder = armEncoder;
        this.limitSwitch = limitSwitch;
    }

    @Override
    public void init() {
        reset();
    }


    private final Map<String, Object> debugMessages = new HashMap<>();
    @Override
    protected void periodic(double dt) {
        final double encoderFactor = ArmConfigs.encoderReversed ? -1:1;
        if (limitSwitch.getSensorReading() != 0)
            this.armEncoderZeroPosition = (int) armEncoder.getSensorReading();
        if (Math.abs(desiredPower) != 0) {
            setArmPower(desiredPower);
            return;
        }
        if (armEncoderZeroPosition == -114514) {
            setArmPower(-0.4);
            return;
        }

        updateDesiredPositions();
        final double armPosition = getArmEncoderPosition(),
                armVelocity = armEncoder.getVelocity() * encoderFactor,
                // armCorrectionPower = armController.getMotorPower(armVelocity, armPosition);
                armCorrectionPower = this.desiredPosition == ArmConfigs.Position.SCORE ?
                        this.simpleArmControllerScoring.getMotorPower(armVelocity, armPosition) + pidBasePower
                        : this.simpleArmControllerNormal.getMotorPower(armVelocity, armPosition);
        debugMessages.put("correction power (by controller)", armCorrectionPower);
        setArmPower(armCorrectionPower);
    }

    private void setArmPower(double power) {
        final double motor1PowerFactor = ArmConfigs.motor1Reversed ? -1:1,
                motor2PowerFactor = ArmConfigs.motor2Reversed? -1:1;
        armMotor1.setPower(power * motor1PowerFactor);
        armMotor2.setPower(power * motor2PowerFactor);
    }

    @Override
    protected void onDestroy() {
        
    }

    @Override
    public void reset() {
        this.desiredPosition = ArmConfigs.Position.INTAKE;
        this.scoringHeight = 1;

        this.armMotor1.getMotorInstance().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.armMotor2.getMotorInstance().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.pidBasePower = this.desiredPower = 0;
    }

    public void setPosition(ArmConfigs.Position position, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        this.desiredPosition = position;
        updateDesiredPositions();
    }

    public boolean isArmInPosition() {
        if (this.desiredPosition == ArmConfigs.Position.INTAKE && limitSwitch.getSensorReading() != 0)
            return true;
        final double errorTolerance = this.desiredPosition == ArmConfigs.Position.SCORE ? ArmConfigs.errorToleranceScoring : ArmConfigs.errorToleranceNormal;
        return Math.abs(getArmEncoderPosition() - getArmDesiredPosition()) < errorTolerance;
    }

    public int getArmEncoderPosition() {
        return (int) ((armEncoder.getSensorReading() - armEncoderZeroPosition) * (ArmConfigs.encoderReversed ? -1: 1));
    }

    public double getArmDesiredPosition() {
        return simpleArmControllerNormal.desiredPosition;
    }

    private void updateDesiredPositions() {
        simpleArmControllerNormal.desiredPosition = desiredPosition == ArmConfigs.Position.SCORE ?
                ArmConfigs.armScoringAnglesAccordingToScoringHeight.getYPrediction(scoringHeight)
                : ArmConfigs.encoderPositions.get(desiredPosition);
    }

    public void setScoringHeight(double scoringHeight, ModulesCommanderMarker operator) {
        System.out.println("arm set scoring height: " + scoringHeight + " by operator: " + operator);
        if (!isOwner(operator))
            return;
        this.scoringHeight = scoringHeight;
        updateDesiredPositions();
    }

    public double getArmDesiredScoringHeight() {
        return this.scoringHeight;
    }

    private double desiredPower, pidBasePower;
    public void forceSetPower(double power, ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;
        desiredPower = power;
    }
    public void setPIDMovingDirection(double direction, ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;
        if (direction <= 0.05)
            pidBasePower = 0;
        else
            pidBasePower = direction > 0 ? ArmConfigs.basePowerWhenMoveUp : ArmConfigs.basePowerWhenMoveDown;
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        debugMessages.put("arm desired position", desiredPosition);
        debugMessages.put("arm desired position (enc)", getArmDesiredPosition());
        debugMessages.put("arm current position (enc)" , getArmEncoderPosition());
        final double encoderFactor = ArmConfigs.encoderReversed ? -1:1;
        debugMessages.put("arm current velocity (enc)", armEncoder.getVelocity() * encoderFactor);
        debugMessages.put("arm in position", isArmInPosition());

        return debugMessages;
    }
}
