package org.firstinspires.ftc.teamcode.Services;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Arm;
import org.firstinspires.ftc.teamcode.Modules.Elevator;
import org.firstinspires.ftc.teamcode.Modules.FlippableDualClaw;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

import java.util.HashMap;
import java.util.Map;

public class UpperStructureService extends RobotService {
    private final Arm arm;
    private final Elevator elevator;
    private final FlippableDualClaw claw;

    private final Gamepad copilotGamePad;

    private enum UpperStructureStatus {
        GRABBING,
        HOLDING,
        SCORING
    }

    private UpperStructureStatus currentStatus;

    public UpperStructureService(Arm arm, Elevator elevator, FlippableDualClaw claw, Gamepad copilotGamePad) {
        this.arm = arm;
        this.elevator = elevator;
        this.claw = claw;
        this.copilotGamePad = copilotGamePad;
    }
    @Override
    public void init() {
        reset();
    }

    @Override
    public void periodic(double dt) {
        keyBindings();

        switch (currentStatus) {
            case HOLDING: {
                claw.setLeftClawClosed(true, this);
                claw.setRightClawClosed(true, this);
                claw.setFlip(false, this);

                arm.setPosition(RobotConfig.ArmConfigs.Position.INTAKE, this);
                break;
            }
            case GRABBING: {
                closeClawOnDemanded();
                openClawOnDemanded();

                claw.setFlip(true, this);

                arm.setPosition(RobotConfig.ArmConfigs.Position.INTAKE, this);
                break;
            }
            case SCORING: {
                openClawOnDemanded();
                arm.setPosition(RobotConfig.ArmConfigs.Position.SCORE, this);
                break;
            }
        }
    }

    private void keyBindings() {
        if (copilotGamePad.y)
            this.currentStatus = UpperStructureStatus.HOLDING;
        if (copilotGamePad.a) {
            this.currentStatus = UpperStructureStatus.GRABBING;
            claw.setLeftClawClosed(false, this);
            claw.setRightClawClosed(false, this);
        }
        if (copilotGamePad.b)
            this.currentStatus = UpperStructureStatus.SCORING;
    }

    private void closeClawOnDemanded() {
        if (copilotGamePad.left_trigger > RobotConfig.ControlConfigs.triggerThreshold)
            claw.setLeftClawClosed(true, this);
        if (copilotGamePad.right_trigger > RobotConfig.ControlConfigs.triggerThreshold)
            claw.setRightClawClosed(true, this);
    }

    private void openClawOnDemanded() {
        if (copilotGamePad.left_bumper)
            claw.setLeftClawClosed(false, this);
        if (copilotGamePad.right_bumper)
            claw.setRightClawClosed(false, this);
    }

    @Override
    public void onDestroy() {

    }

    @Override
    public void reset() {
        claw.gainOwnerShip(this);
        arm.gainOwnerShip(this);
        elevator.gainOwnerShip(this);

        currentStatus = UpperStructureStatus.HOLDING;
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        final Map<String, Object> debugMessages = new HashMap<>();
        debugMessages.put("upper structure status", currentStatus);
        return debugMessages;
    }
}
