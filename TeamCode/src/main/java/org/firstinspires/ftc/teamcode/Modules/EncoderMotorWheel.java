package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.SimpleFeedForwardSpeedController;

import java.util.HashMap;
import java.util.Map;

public class EncoderMotorWheel extends RobotModule {
    private final DcMotorEx motor, quadratureEncoder;
    private final SimpleFeedForwardSpeedController speedController;
    private final double encoderSpeedToPercentSpeed;
    private final HashMap<String, Object> debugMessages = new HashMap<>(1);
    private double desiredSpeed, motorCorrectionRate, encoderCorrectionRate, correctionPower;
    private boolean driveWithEncoder;
    public EncoderMotorWheel(DcMotorEx motor, DcMotorEx quadratureEncoder, SimpleFeedForwardSpeedController speedController, double encoderMaxVelocity) {
        super("encoder wheel", 100);
        this.motor = motor;
        this.quadratureEncoder = quadratureEncoder;
        this.speedController = speedController;
        this.encoderSpeedToPercentSpeed = 1.0/encoderMaxVelocity;
        this.motorCorrectionRate = 1;
        this.encoderCorrectionRate = 1;
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    public void periodic(double dt) {
        if (!super.enabled) {
            motor.setMotorDisable();
            return;
        }

        motor.setMotorEnable();
        this.correctionPower = this.driveWithEncoder ?
                speedController.getSpeedControlPower(quadratureEncoder.getVelocity() * encoderSpeedToPercentSpeed * encoderCorrectionRate, desiredSpeed)
                :desiredSpeed;

        debugMessages.put("actual power", correctionPower * motorCorrectionRate);
        motor.setPower(correctionPower * motorCorrectionRate);
    }

    @Override
    protected void onDestroy() {
        
    }

    @Override
    public void reset() {
        this.desiredSpeed = 0;
        this.driveWithEncoder = RobotConfig.ChassisConfigs.wheelSpeedControlEnabledDefault;
    }

    public void setVelocity(double desiredSpeed) {
        this.desiredSpeed = desiredSpeed;
    }

    public void setMotorReversed(boolean reverseMotor) {
        this.motorCorrectionRate = reverseMotor ? -1:1;
    }

    public void setEncoderReversed(boolean reverseEncoder) {
        this.encoderCorrectionRate = reverseEncoder ? -1:1;
    }

    public void setEncoderEnabled(boolean enabled) {
        this.driveWithEncoder = enabled;
    }

    @Override
    public Map<String, Object> getDebugMessages() {
//        debugMessages.put("desired speed", desiredSpeed);
//        debugMessages.put("correction power", correctionPower);
//        debugMessages.put("actual speed", motor.getVelocity() * encoderSpeedToPercentSpeed * encoderCorrectionRate);
        return debugMessages;
    }
}
