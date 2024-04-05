package org.firstinspires.ftc.teamcode.Services;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.IntakeLegacy;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

public class IntakeServiceLegacy extends RobotService {
    private final IntakeLegacy intake;
    private final PixelDetector pixelDetector;
    private final Gamepad copilotGamepad;
    public IntakeServiceLegacy(IntakeLegacy intake, PixelDetector pixelDetector, Gamepad copilotGamepad) {
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
            intake.setMotion(IntakeLegacy.Motion.ACTIVATED, this);
        else if (copilotGamepad.right_bumper)
            intake.setMotion(IntakeLegacy.Motion.REVERSE, this);
        else intake.setMotion(IntakeLegacy.Motion.STOP, this);
    }

    @Override
    public void onDestroy() {
        intake.setMotion(IntakeLegacy.Motion.STOP, this);
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
