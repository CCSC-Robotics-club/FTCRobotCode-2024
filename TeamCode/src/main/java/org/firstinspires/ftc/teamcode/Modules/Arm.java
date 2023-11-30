package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Utils.Claw;
import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;

import static org.firstinspires.ftc.teamcode.RobotConfig.ArmConfigs;

public class Arm extends RobotModule {
    private final Claw claw;
    private final TouchSensor limitSwitch;
    private final DcMotor armMotor, armEncoder;
    private final double motorPowerRate;
    private final int encoderFactor;

    private int startingEncoderPosition = 0;
    private ArmCommand currentCommand;
    public Arm(Claw claw, DcMotor armMotor, DcMotor armEncoder, TouchSensor limitSwitch) {
        super("arm");
        this.claw = claw;
        this.armMotor = armMotor;
        this.armEncoder = armEncoder;
        this.limitSwitch = limitSwitch;
        this.motorPowerRate = ArmConfigs.armMotorReversed ? -1 : 1 * ArmConfigs.armMotorMaximumPower;
        this.encoderFactor = ArmConfigs.armEncoderReversed ? -1 : 1;
    }

    @Override
    public void init() {
        this.reset();
    }

    @Override
    protected void periodic(double dt) {
        if (limitSwitch.isPressed())
            resetEncoder();

        armMotor.setPower(getMotorPower());
    }

    @Override
    protected void onDestroy() {
        reset();
    }

    @Override
    public void reset() {
        resetEncoder();
        claw.open();
    }

    public void setArmCommand(ArmCommand armCommand, ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;
        this.currentCommand = armCommand;
    }

    private void resetEncoder() {
        startingEncoderPosition = armEncoder.getCurrentPosition() * encoderFactor;
    }

    private int getArmPosition() {
        return armEncoder.getCurrentPosition() * encoderFactor - startingEncoderPosition;
    }

    private double getMotorPower() {
        switch (currentCommand.commandType) {
            case SET_MOTOR_POWER: {
                if (currentCommand.commandValue < 0)
                    if (this.limitSwitch.isPressed() || getArmPosition() <= 0)
                        return 0;
                if (currentCommand.commandValue >0)
                    if (getArmPosition() >= ArmConfigs.positionLimit)
                return currentCommand.commandValue * this.motorPowerRate;
            }
            case SET_POSITION: {
                final double difference = currentCommand.commandValue - getArmPosition();
                if (Math.abs(difference) < ArmConfigs.positionTolerance)
                    return 0;
                final double correctionPower = difference / ArmConfigs.positionDifferenceStartDecelerate * motorPowerRate;
                if (Math.abs(correctionPower) > ArmConfigs.armMotorMaximumPower)
                    return Math.copySign(ArmConfigs.armMotorMaximumPower, correctionPower);
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
            if (this.commandType != ArmCommandType.SET_MOTOR_POWER) {
                if (commandValue > 1)
                    commandValue = 1;
                else if (commandValue < -1)
                    commandValue = -1;
            }
            this.commandValue = commandValue;
        }

    }
}
