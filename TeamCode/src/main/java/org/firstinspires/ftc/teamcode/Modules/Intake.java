package org.firstinspires.ftc.teamcode.Modules;

import static org.firstinspires.ftc.teamcode.RobotConfig.IntakeConfigs;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.RobotModule;

import java.util.HashMap;
import java.util.Map;


public class Intake extends RobotModule {
    // TODO add go-back function, to spew the pixel out of the intake
    private final DcMotor intakeMotor1, intakeMotor2;
    private boolean activated;
    public Intake(DcMotor intakeMotor1, DcMotor intakeMotor2) {
        super("Intake");
        this.intakeMotor1 = intakeMotor1;
        this.intakeMotor2 = intakeMotor2;
    }

    public void setActivated(boolean activated) {
        this.activated = activated;
    }

    @Override
    public void init() {
        this.reset();
    }

    @Override
    protected void periodic(double dt) {
        intakeMotor1.setPower(activated ? IntakeConfigs.intakeMotor1Power : 0);
        intakeMotor2.setPower(activated ? IntakeConfigs.intakeMotor2Power : 0);
    }

    @Override
    protected void onDestroy() {
        this.activated = false;
    }

    @Override
    public void reset() {
        this.activated = false;
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        Map<String, Object> debugMessages = new HashMap<>();

        debugMessages.put("activated", activated);
        return debugMessages;
    }
}
