package org.firstinspires.ftc.teamcode.Services;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Arm;
import org.firstinspires.ftc.teamcode.Modules.Extend;
import org.firstinspires.ftc.teamcode.Modules.FlippableDualClaw;
import org.firstinspires.ftc.teamcode.Robot;
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
        if (copilotGamePad.right_stick_button)
            arm.forceSetPower(copilotGamePad.right_stick_y * -1, this);
        else
            arm.forceSetPower(0, this);
        switch (currentStatus) {
            case HOLDING: {
                claw.setAutoClosing(false, this);
                claw.setLeftClawClosed(true, this);
                claw.setRightClawClosed(true, this);
                if (claw.rightClawInPosition() && claw.leftClawInPosition()) {
                    extend.setExtendPosition(0, this);
                    claw.setFlip(FlippableDualClaw.FlipperPosition.HOLD, this);
                }

                arm.setPosition(RobotConfig.ArmConfigs.Position.INTAKE, this);
                break;
            }
            case GRABBING: {
                claw.setAutoClosing(copilotGamePad.x, this);
                arm.setPosition(RobotConfig.ArmConfigs.Position.INTAKE, this);
                armInPositionDuringCurrentProcess |= arm.isArmInPosition();
                if (armInPositionDuringCurrentProcess)
                    extend.setExtendPosition(RobotConfig.ExtendConfigs.intakeValue, this);
                else
                    extend.setExtendPosition(0, this);

                // auto open claw the moment when arm got in position
                closeClawOnDemanded();
                openClawOnDemanded();
                if (!clawAutoOpenWhenTouchGroundInitiated && arm.isArmInPosition() && claw.flipInPosition()) {
                    claw.setLeftClawClosed(false, this);
                    claw.setRightClawClosed(false, this);
                    clawAutoOpenWhenTouchGroundInitiated = true;
                }
                break;
            }
            case SCORING: {
                claw.setAutoClosing(false, this);

                if (Math.abs(copilotGamePad.left_stick_y) > 0.05)
                    desiredScoringHeight += -0.5 * dt * copilotGamePad.left_stick_y;
                desiredScoringHeight = Math.max(Math.min(desiredScoringHeight, 1), 0);

                chassisService.setDesiredScoringHeight(desiredScoringHeight);
                final double actualScoringHeight = chassisService.getActualScoringHeightAccordingToDistanceToWall(desiredScoringHeight);
                arm.setScoringHeight(actualScoringHeight, this);

                if (armInPositionDuringCurrentProcess) {
                    claw.setScoringAngle(RobotConfig.ArmConfigs.flipperPositionsAccordingToActualArmAngle.getYPrediction(arm.getArmEncoderPosition()), this);
                    extend.setExtendPosition(RobotConfig.ArmConfigs.extendValuesAccordingToActualArmAngle.getYPrediction(arm.getArmEncoderPosition()) / RobotConfig.ExtendConfigs.maxExtendValue, this);
                }
                else {
                    claw.setScoringAngle(1, this);
                    extend.setExtendPosition(0, this);
                }

                /* firstly we close the claw */
                if (!clawRequestedDuringCurrentScoringProcess){
                    claw.setLeftClawClosed(true, this);
                    claw.setRightClawClosed(true, this);
                }
                /* when the claw is closed, we flip the claw and raise the arm */
                if (claw.leftClawInPosition() && claw.rightClawInPosition() && claw.flipInPosition()) {
                    claw.setFlip(FlippableDualClaw.FlipperPosition.SCORE, this);
                    arm.setPosition(RobotConfig.ArmConfigs.Position.SCORE, this);
                    // this.armInPositionDuringCurrentProcess |= arm.isArmInPosition() || chassisService.stickToWallComplete();
                    this.armInPositionDuringCurrentProcess |= arm.isArmInPosition() && chassisService.stickToWallComplete();
                }
                openClawOnDemanded();

                break;
            }
        }
    }

    private boolean clawAutoOpenWhenTouchGroundInitiated = false, clawRequestedDuringCurrentScoringProcess = false;
    private boolean armInPositionDuringCurrentProcess = false;
    private void keyBindings() {
        if (copilotGamePad.y)
            this.currentStatus = UpperStructureStatus.HOLDING;
        if (copilotGamePad.a) {
            this.currentStatus = UpperStructureStatus.GRABBING;
            this.armInPositionDuringCurrentProcess = false;
            claw.setFlip(FlippableDualClaw.FlipperPosition.INTAKE, this);
            clawAutoOpenWhenTouchGroundInitiated = false;
        }
        if (copilotGamePad.b) {
            this.currentStatus = UpperStructureStatus.SCORING;
            // this.desiredScoringHeight = 1;
            this.armInPositionDuringCurrentProcess = false;
            clawRequestedDuringCurrentScoringProcess = false;
        }
    }

    private void closeClawOnDemanded() {
        if (copilotGamePad.left_trigger > RobotConfig.ControlConfigs.triggerThreshold) {
            claw.setLeftClawClosed(true, this);
            this.clawAutoOpenWhenTouchGroundInitiated = true;
        }
        if (copilotGamePad.right_trigger > RobotConfig.ControlConfigs.triggerThreshold) {
            claw.setRightClawClosed(true, this);
            this.clawAutoOpenWhenTouchGroundInitiated = true;
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
        extend.gainOwnerShip(this);

        currentStatus = UpperStructureStatus.HOLDING;
        desiredScoringHeight = 0;
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        final Map<String, Object> debugMessages = new HashMap<>();
        debugMessages.put("upper structure status", currentStatus);
        debugMessages.put("scoring height", desiredScoringHeight);
        return debugMessages;
    }
}
