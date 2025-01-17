package org.firstinspires.ftc.teamcode.Utils.MechanismControllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class EncoderMotorMechanism {
    private final DcMotorEx dcMotorEx;
    private MechanismController controller = null;
    private boolean motorReversed = false, encoderReversed = false;

    public EncoderMotorMechanism(DcMotorEx dcMotorEx) {
        this.dcMotorEx = dcMotorEx;
    }

    public void setPower(double power) {
        dcMotorEx.setPower(power);
    }

    public void setController(MechanismController controller) {
        this.controller = controller;
    }

    public double updateWithController() {
        final double encoderReadingFactor = encoderReversed ? -1:1;
        return updateWithController(encoderReadingFactor * dcMotorEx.getVelocity(), encoderReadingFactor * dcMotorEx.getCurrentPosition());
    }

    public double updateWithController(double mechanismVelocity, double mechanismPosition) {
        if (controller == null) {
            dcMotorEx.setPower(0);
            return 0;
        }
        final double motorPowerFactor = motorReversed ? -1:1,
                power = controller.getMotorPower(mechanismVelocity, mechanismPosition);

        dcMotorEx.setPower(power * motorPowerFactor);
        return power * motorPowerFactor;
    }

    public void setMotorReversed(boolean motorReversed) {
        this.motorReversed = motorReversed;
    }

    public void setEncoderReversed(boolean encoderReversed) {
        this.encoderReversed = encoderReversed;
    }
}
