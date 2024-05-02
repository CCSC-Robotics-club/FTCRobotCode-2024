package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedMotor;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedSensor;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedEncoder;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.SimpleArmController;
import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;

import static org.firstinspires.ftc.teamcode.RobotConfig.ExtendConfigs;

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
        if (extendLimitSwitch.getSensorReading() != 0) {
            this.encoderZeroPosition = extendEncoder.getSensorReading();
            extendMotor.setPower(0);
            return;
        }

        if (encoderZeroPosition == -114514 || controller.desiredPosition <= 0) {
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

        this.controller.desiredPosition = extendPosition * ExtendConfigs.maxExtendValue;
    }

    public boolean isExtendInPosition() {
        return Math.abs(getExtendPosition() - controller.desiredPosition) < ExtendConfigs.errorTolerance;
    }
}
