package org.firstinspires.ftc.teamcode.Services;

import static org.firstinspires.ftc.teamcode.RobotConfig.KeyBindings;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Utils.DriverGamePad;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

public class IntakeService extends RobotService {
    private final Intake intake;
    private final DriverGamePad driverGamePad;
    private final Gamepad copilotGamepad;
    public IntakeService(Intake intake,DriverGamePad driverGamePad, Gamepad copilotGamepad) {
        this.intake = intake;
        this.driverGamePad = driverGamePad;
        this.copilotGamepad = copilotGamepad;
    }
    @Override
    public void init() {
        this.reset();
    }

    @Override
    public void periodic(double dt) {
        intake.setActivated(driverGamePad.keyOnHold(KeyBindings.processAutoIntakePixelButton) || copilotGamepad.x);
    }

    @Override
    public void onDestroy() {
        intake.setActivated(false);
        intake.setEnabled(false);
    }

    @Override
    public void reset() {
        intake.reset();
        intake.gainOwnerShip(this);
    }
}
