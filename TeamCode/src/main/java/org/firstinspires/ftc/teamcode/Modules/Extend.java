package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedMotor;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedSensor;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedEncoder;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.SimpleArmController;
import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;

import static org.firstinspires.ftc.teamcode.RobotConfig.ExtendConfigs;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;
import java.util.Map;

public class Extend extends RobotModule {
    private final ThreadedMotor extendMotor;
    private final ThreadedEncoder extendEncoder;
    private final ThreadedSensor extendLimitSwitch;

    private final SimpleArmController controller;

    private double encoderZeroPosition;
    public Extend(ThreadedMotor extendMotor, ThreadedEncoder extendEncoder, ThreadedSensor extendLimitSwitch) {
        super("extend");
        this.extendMotor = extendMotor;
        this.extendEncoder = extendEncoder;
        this.extendLimitSwitch = extendLimitSwitch;

        controller = new SimpleArmController(
                ExtendConfigs.maxPowerWhenMovingForward,
                ExtendConfigs.maxPowerWhenMovingBackward,
                ExtendConfigs.errorStartDecelerate,
                ExtendConfigs.powerNeededToMoveForward,
                ExtendConfigs.powerNeededToMoveBackward,
                ExtendConfigs.errorTolerance,
                false
        );
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    protected void periodic(double dt) {
        final double motorPowerFactor = ExtendConfigs.extendMotorReversed ? -1 : 1,
                encoderFactor = ExtendConfigs.extendEncoderReversed ? -1 : 1;
        if (extendLimitSwitch.getSensorReading() != 0 && controller.desiredPosition == 0) {
            this.encoderZeroPosition = extendEncoder.getSensorReading();
            extendMotor.setPower(0);
            return;
        }

        if (encoderZeroPosition == -114514 || controller.desiredPosition == 0) {
            extendMotor.setPower(-0.4 * motorPowerFactor);
            return;
        }

        final double correctionPower = controller.getMotorPower(extendEncoder.getVelocity() * encoderFactor, getExtendPosition());
        extendMotor.setPower(correctionPower * motorPowerFactor);
    }

    @Override
    protected void onDestroy() {

    }

    @Override
    public void reset() {
        this.encoderZeroPosition = -114514;
        this.controller.desiredPosition = 0;
        this.extendMotor.getMotorInstance().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double getExtendPosition() {
        final double encoderFactor = ExtendConfigs.extendEncoderReversed ? -1 : 1;
        return (this.extendEncoder.getSensorReading() - this.encoderZeroPosition) * encoderFactor;
    }

    /**
     * @param extendPosition 0~1
     * */
    public void setExtendPosition(double extendPosition, ModulesCommanderMarker operator) {
        if (!isOwner(operator)) return;
        if (encoderZeroPosition == -114514) return;

        this.controller.desiredPosition = Math.max(0, Math.min(ExtendConfigs.maxExtendValue, extendPosition));
    }

    public boolean isExtendInPosition() {
        return Math.abs(getExtendPosition() - controller.desiredPosition) < ExtendConfigs.errorAsTaskFinished;
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        final Map<String, Object> debugMessages = new HashMap<>();
        debugMessages.put("desired extend position", controller.desiredPosition);
        debugMessages.put("extend limit reading", extendLimitSwitch.getSensorReading());
        debugMessages.put("zero position", encoderZeroPosition);
        return debugMessages;
    }
}
