package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.SimpleArmController;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.MotorThreaded;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.RobotService;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.SimpleSensor;

import static org.firstinspires.ftc.teamcode.RobotConfig.ArmConfigs;


import java.util.HashMap;
import java.util.Map;


public class Arm extends RobotModule {
    private final MotorThreaded armMotor;
    private final SimpleSensor armEncoder;
    private final SimpleSensor limitSwitch;
    private int armEncoderZeroPosition = -114514;
    private double scoringHeight;
    private long lastLimitSwitchActivationTime;

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
    public Arm(MotorThreaded armMotor, SimpleSensor armEncoder, SimpleSensor limitSwitch) {
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

        if (limitSwitch.getSensorReading() != 0) {
            this.lastLimitSwitchActivationTime = System.currentTimeMillis();
            this.armEncoderZeroPosition = (int) armEncoder.getSensorReading();
        }

        if (armStuck() || Math.abs(desiredPower) > 0.05) {
            armMotor.setPower(motorPowerFactor * desiredPower);
            return;
        }
        if (armEncoderZeroPosition == -114514) {
            armMotor.setPower(-0.6 * motorPowerFactor);
            return;
        }

        armController.desiredPosition = ArmConfigs.encoderPositions.get(desiredPosition);
        if (desiredPosition == ArmConfigs.Position.INTAKE) {
            armMotor.setPower(limitSwitch.getSensorReading() != 0 ? 0 : motorPowerFactor * armController.getMotorPower(0, getArmEncoderPosition() + 100));
            return;
        }
        if (desiredPosition == ArmConfigs.Position.SCORE)
            armController.desiredPosition = ArmConfigs.armScoringAnglesAccordingToScoringHeight.getYPrediction(scoringHeight);

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

        lastLimitSwitchActivationTime = System.currentTimeMillis();
        this.armMotor.getMotorInstance().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPosition(ArmConfigs.Position position, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        if (this.desiredPosition != ArmConfigs.Position.INTAKE && position == ArmConfigs.Position.INTAKE)
            this.lastLimitSwitchActivationTime = System.currentTimeMillis();
        this.desiredPosition = position;
    }

    public boolean isArmInPosition() {
        return Math.abs(getArmEncoderPosition() - armController.desiredPosition) < ArmConfigs.errorTolerance * 2;
    }

    public int getArmEncoderPosition() {
        return (int) ((armEncoder.getSensorReading() - armEncoderZeroPosition) * (ArmConfigs.encoderReversed ? -1: 1));
    }

    public void setScoringHeight(double scoringHeight, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        this.scoringHeight = scoringHeight;
    }

    public double getArmScoringHeight() {
        return this.scoringHeight;
    }

    public boolean armStuck() {
        return limitSwitch.getSensorReading() == 0 && System.currentTimeMillis() - lastLimitSwitchActivationTime > 5000 && this.desiredPosition == ArmConfigs.Position.INTAKE;
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
