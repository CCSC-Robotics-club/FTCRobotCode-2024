package org.firstinspires.ftc.teamcode.Modules;

import static org.firstinspires.ftc.teamcode.RobotConfig.IntakeConfigs;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;

import java.util.HashMap;
import java.util.Map;


public class Intake extends RobotModule {
    // TODO add go-back function, to spew the pixel out of the intake
    private final DcMotor intakeMotor1, intakeMotor2;
    private Motion motion;
    public Intake(DcMotor intakeMotor1, DcMotor intakeMotor2) {
        super("Intake");
        this.intakeMotor1 = intakeMotor1;
        this.intakeMotor2 = intakeMotor2;
    }

    public void setMotion(Motion motion, ModulesCommanderMarker operator) {
        if (isOwner(operator)) this.motion = motion;
    }

    @Override
    public void init() {
        this.reset();
    }

    @Override
    protected void periodic(double dt) {
        intakeMotor1.setPower(IntakeConfigs.intakeMotor1Power * getMotorPower());
        intakeMotor2.setPower(IntakeConfigs.intakeMotor2Power * getMotorPower());
    }

    private double getMotorPower() {
        switch (motion) {
            case ACTIVATED: return 1;
            case REVERSE: return -1;
            default: return 0;
        }
    }

    @Override
    protected void onDestroy() {
        this.motion = Motion.STOP;
    }

    @Override
    public void reset() {
        this.motion = Motion.STOP;
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        Map<String, Object> debugMessages = new HashMap<>();

        debugMessages.put("motion", motion);
        return debugMessages;
    }

    public enum Motion {
        STOP,
        ACTIVATED,
        REVERSE
    }
}
