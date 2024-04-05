package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;

import static org.firstinspires.ftc.teamcode.RobotConfig.ArmConfigsLegacy;

import java.util.HashMap;
import java.util.Map;

public class ArmLegacy extends RobotModule {
    private final ExtendableClawLegacy claw;
    private final TouchSensor limitSwitch;
    private final DcMotor armMotor1, armMotor2, armEncoder;
    private final double motor1PowerRate, motor2PowerRate;
    private final int encoderFactor;

    private int startingEncoderPosition = 0;
    private ArmCommand currentCommand;
    public ArmLegacy(DcMotor armMotor1, DcMotor armMotor2, DcMotor armEncoder, ExtendableClawLegacy extendableClaw, TouchSensor limitSwitch) {
        super("arm");
        this.claw = extendableClaw;
        this.armMotor1 = armMotor1;
        this.armMotor2 = armMotor2;
        this.armEncoder = armEncoder;
        this.limitSwitch = limitSwitch;
        this.motor1PowerRate = ArmConfigsLegacy.armMotor1Reversed ? -1 : 1 * ArmConfigsLegacy.armMotorMaximumPower;
        this.motor2PowerRate = ArmConfigsLegacy.armMotor2Reversed ? -1:1 * ArmConfigsLegacy.armMotorMaximumPower;
        this.encoderFactor = ArmConfigsLegacy.armEncoderReversed ? -1 : 1;
    }

    @Override
    public void init() {
        this.reset();
    }

    @Override
    protected void periodic(double dt) {
        if (limitSwitch.isPressed())
            resetEncoder();

        final double motorPower = getMotorPower();
        armMotor1.setPower(motorPower * motor1PowerRate);
        armMotor2.setPower(motorPower * motor2PowerRate);
        debugMessages.put("command type", currentCommand.commandType);
        debugMessages.put("command value", currentCommand.commandValue);
        debugMessages.put("motor power", getMotorPower());
    }

    @Override
    protected void onDestroy() {
        reset();
    }

    @Override
    public void reset() {
        resetEncoder();
        claw.reset();
        claw.gainOwnerShip(this);
        currentCommand = ArmConfigsLegacy.armCommandWhenNoInput;
    }

    public void setArmCommand(ArmCommand armCommand, ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;
        this.currentCommand = armCommand;
    }

    public void placePixel(ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;
        this.claw.placePixelToBoard(this);
    }

    public void holdPixel(ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;
        this.claw.holdPixel(this);
    }

    private void resetEncoder() {
        startingEncoderPosition = armEncoder.getCurrentPosition() * encoderFactor;
    }

    private int getArmPosition() {
        return armEncoder.getCurrentPosition() * encoderFactor - startingEncoderPosition;
    }

    private boolean isLimitSwitchPressed() {
        return limitSwitch != null && limitSwitch.isPressed();
    }

    public boolean isArmDesiredPositionReached() {
        if (this.currentCommand.commandType == ArmCommand.ArmCommandType.SET_MOTOR_POWER)
            return true;
        return Math.abs(getArmPosition() - currentCommand.commandValue) < ArmConfigsLegacy.positionErrorAsCommandFinished;
    }

    private double getMotorPower() {
        debugMessages.put("encoder reading", getArmPosition());
        switch (currentCommand.commandType) {
            case SET_MOTOR_POWER: {
                if (currentCommand.commandValue < 0)
                    if (isLimitSwitchPressed() || getArmPosition() < 0)
                        return 0;
                if (currentCommand.commandValue > 0)
                    if (getArmPosition() > ArmConfigsLegacy.positionLimit)
                        return 0;
                return currentCommand.commandValue;
            }
            case SET_POSITION: {
                final double difference = currentCommand.commandValue - getArmPosition();
                if (Math.abs(difference) < ArmConfigsLegacy.positionTolerance)
                    return 0;
                final double correctionPower = difference / ArmConfigsLegacy.positionDifferenceStartDecelerate;
                if (Math.abs(correctionPower) > ArmConfigsLegacy.armMotorMaximumPower)
                    return Math.copySign(ArmConfigsLegacy.armMotorMaximumPower, correctionPower);
                if (Math.abs(correctionPower) < ArmConfigsLegacy.frictionPower)
                    return Math.copySign(ArmConfigsLegacy.frictionPower, correctionPower);
                return correctionPower;
            }
        }
        throw new IllegalArgumentException("unknown command type"+ currentCommand.commandType.name());
    }

    public static final class ArmCommand {
        public enum ArmCommandType {
            SET_POSITION,
            SET_MOTOR_POWER
        }
        public final ArmCommandType commandType;
        public final double commandValue;

        public ArmCommand(ArmCommandType commandType, double commandValue) {
            this.commandType = commandType;
            if (this.commandType == ArmCommandType.SET_MOTOR_POWER) {
                if (commandValue > 1)
                    commandValue = 1;
                else if (commandValue < -1)
                    commandValue = -1;
            }
            this.commandValue = commandValue;
        }

    }

    private final Map<String, Object> debugMessages = new HashMap<>(1);
    @Override
    public Map<String, Object> getDebugMessages() {
        return debugMessages;
    }
}
