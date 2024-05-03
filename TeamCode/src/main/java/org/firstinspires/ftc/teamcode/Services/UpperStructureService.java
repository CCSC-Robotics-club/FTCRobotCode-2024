package org.firstinspires.ftc.teamcode.Services;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Arm;
import org.firstinspires.ftc.teamcode.Modules.Extend;
import org.firstinspires.ftc.teamcode.Modules.FlippableDualClaw;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

import java.util.HashMap;
import java.util.Map;

public class UpperStructureService extends RobotService {
    private final Arm arm;
    private final Extend extend;
    private final FlippableDualClaw claw;
    private final PilotChassisService chassisService;

    private final Gamepad copilotGamePad;

    private double desiredScoringHeight;

    private enum UpperStructureStatus {
        GRABBING,
        HOLDING,
        SCORING
    }

    private UpperStructureStatus currentStatus;

    public UpperStructureService(Arm arm, Extend extend, FlippableDualClaw claw, PilotChassisService chassisService, Gamepad copilotGamePad) {
        this.arm = arm;
        this.extend = extend;
        this.claw = claw;
        this.chassisService = chassisService;
        this.copilotGamePad = copilotGamePad;
    }
    @Override
    public void init() {
        reset();
    }

    @Override
    public void periodic(double dt) {
        keyBindings();
        arm.forceSetPower(copilotGamePad.right_stick_y * -1, this);
        switch (currentStatus) {
            case HOLDING: {
                claw.setAutoClosing(false, this);
                claw.setScoringAngle(RobotConfig.FlippableDualClawConfigs.flipperHoldPosition, this);
                claw.setLeftClawClosed(true, this);
                claw.setRightClawClosed(true, this);
                if (claw.rightClawInPosition() && claw.leftClawInPosition())
                    claw.setFlip(false, this);

                arm.setPosition(RobotConfig.ArmConfigs.Position.INTAKE, this);
                extend.setExtendPosition(0, this);
                break;
            }
            case GRABBING: {
                claw.setAutoClosing(copilotGamePad.x, this);
                extend.setExtendPosition(0, this);
                arm.setPosition(RobotConfig.ArmConfigs.Position.INTAKE, this);

                closeClawOnDemanded();
                openClawOnDemanded();

                // auto open claw the moment when arm got in position
                if (!clawAutoOpenWhenTouchGroundInitated && arm.isArmInPosition() && claw.flipInPosition()) {
                    claw.setLeftClawClosed(false, this);
                    claw.setRightClawClosed(false, this);
                    clawAutoOpenWhenTouchGroundInitated = true;
                }
                break;
            }
            case SCORING: {
                claw.setAutoClosing(false, this);

                if (Math.abs(copilotGamePad.left_stick_y) > 0.05)
                    desiredScoringHeight += -1 * dt * copilotGamePad.left_stick_y;
                desiredScoringHeight = Math.max(Math.min(desiredScoringHeight, 2), 0);

                chassisService.setDesiredScoringHeight(Math.min(1, desiredScoringHeight));
                final double actualScoringHeight = chassisService.getActualScoringHeightAccordingToDistanceToWall(1);
                if (desiredScoringHeight <= 1) {
                    arm.setScoringHeight(actualScoringHeight, this);
                    claw.setScoringAngle(RobotConfig.ArmConfigs.flipperPositionsAccordingToScoringHeightNormal.getYPrediction(actualScoringHeight), this);
                    extend.setExtendPosition(RobotConfig.ArmConfigs.extendValueDuringNormalScoring, this);
                } else {
                    arm.setScoringHeight(desiredScoringHeight, this);
                    claw.setScoringAngle(RobotConfig.ArmConfigs.flipperPositionsAccordingToScoringHeightExtended.getYPrediction(desiredScoringHeight), this);
                    extend.setExtendPosition(RobotConfig.ArmConfigs.extendValueAccordingToScoringHeight.getYPrediction(desiredScoringHeight), this);
                }

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
    }

    private boolean clawAutoOpenWhenTouchGroundInitated = false, clawRequestedDuringCurrentScoringProcess = false;
    private void keyBindings() {
        if (copilotGamePad.y)
            this.currentStatus = UpperStructureStatus.HOLDING;
        if (copilotGamePad.a) {
            this.currentStatus = UpperStructureStatus.GRABBING;
            claw.setFlip(true, this);
            clawAutoOpenWhenTouchGroundInitated = false;
        }
        if (copilotGamePad.b) {
            this.currentStatus = UpperStructureStatus.SCORING;
            clawRequestedDuringCurrentScoringProcess = false;
        }
    }

    private void closeClawOnDemanded() {
        if (copilotGamePad.left_trigger > RobotConfig.ControlConfigs.triggerThreshold)
            claw.setLeftClawClosed(true, this);
        if (copilotGamePad.right_trigger > RobotConfig.ControlConfigs.triggerThreshold)
            claw.setRightClawClosed(true, this);
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
        extend.gainOwnerShip(this);

        currentStatus = UpperStructureStatus.HOLDING;
        desiredScoringHeight = 1;
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        final Map<String, Object> debugMessages = new HashMap<>();
        debugMessages.put("upper structure status", currentStatus);
        debugMessages.put("scoring height", desiredScoringHeight);
        return debugMessages;
    }
}
