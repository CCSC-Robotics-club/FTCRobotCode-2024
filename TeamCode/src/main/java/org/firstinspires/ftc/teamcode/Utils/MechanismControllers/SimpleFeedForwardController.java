package org.firstinspires.ftc.teamcode.Utils.MechanismControllers;

import org.firstinspires.ftc.teamcode.Utils.MathUtils.LookUpTable;

public class SimpleFeedForwardController implements MechanismController {
    private final LookUpTable motorPowerLookUpTable;
    private double desiredSpeed = 0;
    public SimpleFeedForwardController(LookUpTable motorPowerLookUpTable) {
        this.motorPowerLookUpTable = motorPowerLookUpTable;
    }

    public void setDesiredSpeed(double desiredSpeed) {
        this.desiredSpeed = desiredSpeed;
    }
    @Override
    public double getMotorPower(double mechanismVelocity, double mechanismPosition) {
        return Math.copySign(motorPowerLookUpTable.getYPrediction(Math.abs(desiredSpeed)), desiredSpeed);
    }
}
