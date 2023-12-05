package org.firstinspires.ftc.teamcode.Services;

import static org.firstinspires.ftc.teamcode.RobotConfig.KeyBindings;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.DriverGamePad;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

public class IntakeService extends RobotService {
    private final Intake intake;
    private final PixelDetector pixelDetector;
    private final Gamepad copilotGamepad;
    public IntakeService(Intake intake, PixelDetector pixelDetector, Gamepad copilotGamepad) {
        this.intake = intake;
        this.pixelDetector = pixelDetector;
        this.copilotGamepad = copilotGamepad;
    }
    @Override
    public void init() {
        this.reset();
    }

    @Override
    public void periodic(double dt) {
        if (pixelDetector.isPixelInFront()
                || copilotGamepad.right_trigger > RobotConfig.ControlConfigs.triggerThreshold)
            intake.setMotion(Intake.Motion.ACTIVATED, this);
        else if (copilotGamepad.right_bumper)
            intake.setMotion(Intake.Motion.REVERSE, this);
        else intake.setMotion(Intake.Motion.STOP, this);
    }

    @Override
    public void onDestroy() {
        intake.setMotion(Intake.Motion.STOP, this);
        intake.setEnabled(false);
    }

    @Override
    public void reset() {
        intake.reset();
        intake.gainOwnerShip(this);
    }

    public interface PixelDetector {
        boolean isPixelInFront();
    }
}
