package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedEncoder;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.ArmGravityController;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedMotor;
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
    private final ArmGravityController armController = new ArmGravityController(ArmConfigs.armProfile);

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
        final double motor1PowerFactor = ArmConfigs.motor1Reversed ? -1:1,
                motor2PowerFactor = ArmConfigs.motor2Reversed? -1:1,
                encoderFactor = ArmConfigs.encoderReversed ? -1:1;
        if (limitSwitch.getSensorReading() != 0)
            this.armEncoderZeroPosition = (int) armEncoder.getSensorReading();
        if (Math.abs(desiredPower) > 0.05) {
            armMotor1.setPower(desiredPower * motor1PowerFactor);
            armMotor2.setPower(desiredPower * motor2PowerFactor);
            return;
        }
        if (armEncoderZeroPosition == -114514) {
            armMotor1.setPower(0);
            armMotor2.setPower(0);
            return;
        }

        armController.goToDesiredPosition(ArmConfigs.encoderPositions.get(desiredPosition));
        if (desiredPosition == ArmConfigs.Position.INTAKE &&
                (limitSwitch.getSensorReading()!=0 || Math.abs(getArmEncoderPosition() - armController.getDesiredPosition()) < ArmConfigs.armProfile.staticPIDProfile.getErrorTolerance())) {
            armMotor1.setPower(0);
            armMotor2.setPower(0);
            return;
        }

        if (desiredPosition == ArmConfigs.Position.SCORE)
            armController.updateDesiredPosition(
                    scoringHeight < 1 ?
                            ArmConfigs.armScoringAnglesAccordingToScoringHeightNormal.getYPrediction(scoringHeight)
                            : ArmConfigs.armScoringAnglesAccordingToScoringHeightExtended.getYPrediction(scoringHeight)
            );

        final double armPosition = getArmEncoderPosition(),
                armVelocity = armEncoder.getVelocity() * encoderFactor,
                armCorrectionPower = armController.getMotorPower(armVelocity, armPosition);

        debugMessages.put("correction power (by controller)", armCorrectionPower);

        armMotor1.setPower(armCorrectionPower * motor1PowerFactor);
        armMotor2.setPower(armCorrectionPower * motor2PowerFactor);
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
    }

    public void setPosition(ArmConfigs.Position position, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        this.desiredPosition = position;
    }

    public boolean isArmInPosition() {
        if (this.desiredPosition == ArmConfigs.Position.INTAKE)
            return limitSwitch.getSensorReading() != 0;
        return Math.abs(armEncoder.getSensorReading() - armController.getDesiredPosition()) < ArmConfigs.errorAsArmInPosition;
    }

    public int getArmEncoderPosition() {
        return (int) ((armEncoder.getSensorReading() - armEncoderZeroPosition) * (ArmConfigs.encoderReversed ? -1: 1));
    }

    public void setScoringHeight(double scoringHeight, ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;
        this.scoringHeight = scoringHeight;
    }

    public double getArmScoringHeight() {
        return this.scoringHeight;
    }

    private double desiredPower = 0;
    public void forceSetPower(double power, ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;
        desiredPower = power;
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
