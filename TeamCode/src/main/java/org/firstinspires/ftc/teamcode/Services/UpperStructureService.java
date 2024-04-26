package org.firstinspires.ftc.teamcode.Services;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Arm;
import org.firstinspires.ftc.teamcode.Modules.FlippableDualClaw;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

import java.util.HashMap;
import java.util.Map;

public class UpperStructureService extends RobotService {
    private final Arm arm;
    private final FlippableDualClaw claw;

    private final Gamepad copilotGamePad;

    private double scoringHeight;

    private enum UpperStructureStatus {
        GRABBING,
        HOLDING,
        SCORING
    }

    private UpperStructureStatus currentStatus;

    public UpperStructureService(Arm arm, FlippableDualClaw claw, Gamepad copilotGamePad) {
        this.arm = arm;
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
                claw.setAutoClosing(false, this);
                claw.setLeftClawClosed(true, this);
                claw.setRightClawClosed(true, this);
                if (claw.rightClawInPosition() && claw.leftClawInPosition())
                    claw.setFlip(false, this);

                arm.setPosition(RobotConfig.ArmConfigs.Position.INTAKE, this);
                break;
            }
            case GRABBING: {
                claw.setAutoClosing(copilotGamePad.x, this);
                arm.setPosition(RobotConfig.ArmConfigs.Position.INTAKE, this);
                claw.setFlip(true, this);
                closeClawOnDemanded();
                openClawOnDemanded();

                // auto open claw the moment when arm got in position
                if (!clawRequestedDuringCurrentGrabbingProcess && arm.isArmInPosition() && claw.flipInPosition()) {
                    claw.setLeftClawClosed(false, this);
                    claw.setRightClawClosed(false, this);
                }
                break;
            }
            case SCORING: {
                claw.setAutoClosing(false, this);
                arm.setScoringHeight(scoringHeight, this);
                if (Math.abs(copilotGamePad.left_stick_y) > 0.05)
                    scoringHeight += -1 * dt * copilotGamePad.left_stick_y;
                scoringHeight = Math.max(Math.min(scoringHeight, 0.85), 0.25);
                /* firstly we close the claw */
                if (!clawRequestedDuringCurrentScoringProcess){
                    claw.setLeftClawClosed(true, this);
                    claw.setRightClawClosed(true, this);
                }
                /* when the claw is closed, we flip the claw and raise the arm */
                if (claw.leftClawInPosition() && claw.rightClawInPosition() && claw.flipInPosition()) {
                    claw.setFlip(false, this);
                    arm.setPosition(RobotConfig.ArmConfigs.Position.SCORE, this);
                }
                /* when arm reaches position, we can close/open claw on pilot demand */
                if (arm.isArmInPosition())
                    openClawOnDemanded();

                break;
            }
        }

        if (arm.armStuck())
            copilotGamePad.rumble(100);

        arm.forceSetPower(-copilotGamePad.right_stick_y, this);
    }

    private boolean clawRequestedDuringCurrentGrabbingProcess = false, clawRequestedDuringCurrentScoringProcess = false;
    private void keyBindings() {
        if (copilotGamePad.y)
            this.currentStatus = UpperStructureStatus.HOLDING;
        if (copilotGamePad.a) {
            this.currentStatus = UpperStructureStatus.GRABBING;
            clawRequestedDuringCurrentGrabbingProcess = false;
        }
        if (copilotGamePad.b) {
            this.currentStatus = UpperStructureStatus.SCORING;
            clawRequestedDuringCurrentScoringProcess = false;
            this.scoringHeight = 0.85;
        }
    }

    private void closeClawOnDemanded() {
        if (copilotGamePad.left_trigger > RobotConfig.ControlConfigs.triggerThreshold) {
            clawRequestedDuringCurrentGrabbingProcess = true;
            claw.setLeftClawClosed(true, this);
        }
        if (copilotGamePad.right_trigger > RobotConfig.ControlConfigs.triggerThreshold) {
            clawRequestedDuringCurrentGrabbingProcess = true;
            claw.setRightClawClosed(true, this);
        }
    }

    private void openClawOnDemanded() {
        if (copilotGamePad.left_bumper) {
            clawRequestedDuringCurrentScoringProcess = true;
            claw.setLeftClawClosed(false, this);
        }
        if (copilotGamePad.right_bumper) {
            clawRequestedDuringCurrentScoringProcess = true;
            claw.setRightClawClosed(false, this);
        }
    }

    @Override
    public void onDestroy() {

    }

    @Override
    public void reset() {
        claw.gainOwnerShip(this);
        arm.gainOwnerShip(this);

        currentStatus = UpperStructureStatus.HOLDING;
        scoringHeight = 1;
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        final Map<String, Object> debugMessages = new HashMap<>();
        debugMessages.put("upper structure status", currentStatus);
        debugMessages.put("scoring height", scoringHeight);
        return debugMessages;
    }
}
