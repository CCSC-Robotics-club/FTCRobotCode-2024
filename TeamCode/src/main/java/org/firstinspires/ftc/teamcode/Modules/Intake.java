package org.firstinspires.ftc.teamcode.Modules;

import static org.firstinspires.ftc.teamcode.RobotConfig.IntakeConfigs;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.RobotModule;

public class Intake extends RobotModule {
    private final DcMotor intakeMotor1, intakeMotor2;
    public Intake(DcMotor intakeMotor1, DcMotor intakeMotor2) {
        super("Intake");
        this.intakeMotor1 = intakeMotor1;
        this.intakeMotor2 = intakeMotor2;
    }

    @Override
    public void init() {

    }

    @Override
    protected void periodic(double dt) {

    }

    @Override
    protected void onDestroy() {

    }

    @Override
    public void reset() {

    }
}
