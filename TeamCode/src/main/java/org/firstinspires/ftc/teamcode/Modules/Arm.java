package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedEncoder;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.ArmGravityController;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.MechanismController;
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
    private final ThreadedEncoder armEncoder;
    private final SimpleSensor limitSwitch;
    private int armEncoderZeroPosition = -114514;
    private double scoringHeight;
    private final ArmGravityController armController = new ArmGravityController(ArmConfigs.armProfile);

    private ArmConfigs.Position desiredPosition;
    public Arm(MotorThreaded armMotor, ThreadedEncoder armEncoder, SimpleSensor limitSwitch) {
        super("arm");
        this.armMotor = armMotor;
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
        final double motorPowerFactor = ArmConfigs.motorReversed ? -1:1,
                encoderFactor = ArmConfigs.encoderReversed ? -1:1;
        if (limitSwitch.getSensorReading() != 0)
            this.armEncoderZeroPosition = (int) armEncoder.getSensorReading();
        if (armEncoderZeroPosition == -114514) {
            armMotor.setPower(0);
            return;
        }

        armController.goToDesiredPosition(ArmConfigs.encoderPositions.get(desiredPosition));
        if (desiredPosition == ArmConfigs.Position.INTAKE && (limitSwitch.getSensorReading()!=0 || isArmInPosition())) {
            armMotor.setPower(0);
            return;
        }

        if (desiredPosition == ArmConfigs.Position.SCORE)
            armController.updateDesiredPosition(ArmConfigs.armScoringAnglesAccordingToScoringHeight.getYPrediction(scoringHeight));

        final double armPosition = getArmEncoderPosition(),
                armVelocity = armEncoder.getVelocity() * encoderFactor,
                armCorrectionPower = armController.getMotorPower(armVelocity, armPosition);

        debugMessages.put("correction power (by controller)", armCorrectionPower);

        debugMessages.put("arm final correction power", armCorrectionPower * motorPowerFactor);
        armMotor.setPower(armCorrectionPower * motorPowerFactor);
    }

    @Override
    protected void onDestroy() {
        
    }

    @Override
    public void reset() {
        this.desiredPosition = ArmConfigs.Position.INTAKE;
        this.scoringHeight = 1;

        this.armMotor.getMotorInstance().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPosition(ArmConfigs.Position position, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        this.desiredPosition = position;
    }

    public boolean isArmInPosition() {
        if (this.desiredPosition == ArmConfigs.Position.INTAKE)
            return limitSwitch.getSensorReading() != 0;
        return Math.abs((double) armEncoder.getSensorReading() - armController.getDesiredPosition()) < ArmConfigs.armProfile.staticPIDProfile.getErrorTolerance();
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

    @Override
    public Map<String, Object> getDebugMessages() {
        debugMessages.put("arm desired position", desiredPosition);
        debugMessages.put("arm desired encoder position", armController.getDesiredPosition());
        debugMessages.put("arm current position (enc)" , getArmEncoderPosition());
        final double encoderFactor = ArmConfigs.encoderReversed ? -1:1;
        debugMessages.put("arm current velocity (enc)", armEncoder.getVelocity() * encoderFactor);
        debugMessages.put("arm in position", isArmInPosition());

        return debugMessages;
    }
}
