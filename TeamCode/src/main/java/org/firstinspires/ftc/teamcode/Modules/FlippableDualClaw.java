package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedLED;
import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedMotor;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ProfiledServo;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.RobotService;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedSensor;

import static org.firstinspires.ftc.teamcode.RobotConfig.FlippableDualClawConfigs;

import java.util.HashMap;
import java.util.Map;

public class FlippableDualClaw extends RobotModule {
    public enum FlipperPosition {
        INTAKE,
        PREPARE_TO_GRAB_STACK,
        HOLD,
        SCORE
    }
    public final Map<FlipperPosition, Double> flipperPositions = new HashMap<>();


    private final ProfiledServo flip, leftClaw, rightClaw;
    private final ThreadedLED leftIndicatorLight, rightIndicatorLight;
    private final ThreadedSensor detectorLeft, detectorRight;

    private boolean closeLeftClaw, closeRightClaw, autoClosing;
    private FlipperPosition flipperPosition;

    public FlippableDualClaw(ProfiledServo flip, ProfiledServo leftClaw, ProfiledServo rightClaw, ThreadedSensor detectorLeft, ThreadedSensor detectorRight, ThreadedLED leftIndicatorLight, ThreadedLED rightIndicatorLight) {
        super("claw");
        this.flip = flip;
        this.leftClaw = leftClaw;
        this.rightClaw = rightClaw;

        this.detectorLeft = detectorLeft;
        this.detectorRight = detectorRight;
        this.leftIndicatorLight = leftIndicatorLight;
        this.rightIndicatorLight = rightIndicatorLight;

        flipperPositions.put(FlipperPosition.INTAKE, 0.02);
        flipperPositions.put(FlipperPosition.PREPARE_TO_GRAB_STACK, 0.3);
        flipperPositions.put(FlipperPosition.HOLD, 0.7);
        flipperPositions.put(FlipperPosition.SCORE, 1.0);
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    protected void periodic(double dt) {
        detectorLeft.setEnabled(autoClosing);
        detectorRight.setEnabled(autoClosing);

        final boolean leftClawDetected = detectorLeft.getSensorReading() > FlippableDualClawConfigs.colorDetectorThreshold,
                rightClawDetected = detectorRight.getSensorReading() > FlippableDualClawConfigs.colorDetectorThreshold;

        closeLeftClaw |= autoClosing && leftClawDetected;
        closeRightClaw |= autoClosing && rightClawDetected;

        leftClaw.setDesiredPosition(closeLeftClaw ? FlippableDualClawConfigs.leftClawClosePosition : FlippableDualClawConfigs.leftClawOpenPosition);
        rightClaw.setDesiredPosition(closeRightClaw ? FlippableDualClawConfigs.rightClawClosedPosition : FlippableDualClawConfigs.rightClawOpenPosition);

        flip.setDesiredPosition(flipperPositions.get(flipperPosition));

        /* flip automatically */
        if (flipperPosition == FlipperPosition.INTAKE && autoClosing && leftClawDetected && rightClawDetected && leftClaw.inPosition() && rightClaw.inPosition())
            this.flipperPosition = FlipperPosition.HOLD;

        leftClaw.update(dt);
        rightClaw.update(dt);
        flip.update(dt);

        final boolean blink = Math.sin(System.currentTimeMillis() / 1000.0f * 2) > 0;
        leftIndicatorLight.setEnabled(leftClawDetected && (!autoClosing || blink));
        rightIndicatorLight.setEnabled(rightClawDetected && (!autoClosing || blink));
    }

    @Override
    protected void onDestroy() {

    }

    @Override
    public void reset() {
        closeLeftClaw = closeRightClaw = autoClosing = false;
        this.flipperPosition = FlipperPosition.HOLD;

        leftIndicatorLight.setEnabled(false);
        rightIndicatorLight.setEnabled(false);

        flip.setDesiredPosition(flipperPositions.get(FlipperPosition.HOLD));
        flip.update(10);
        leftClaw.setDesiredPosition((FlippableDualClawConfigs.leftClawClosePosition + FlippableDualClawConfigs.leftClawOpenPosition)/2);
        leftClaw.update(10);
        rightClaw.setDesiredPosition((FlippableDualClawConfigs.rightClawClosedPosition + FlippableDualClawConfigs.rightClawOpenPosition)/2);
        rightClaw.update(10);
    }

    public void setLeftClawClosed(boolean close, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        this.closeLeftClaw = close;
        periodic(0); // flush claw status
    }

    public void setRightClawClosed(boolean close, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        this.closeRightClaw = close;
        periodic(0); // flush claw status
    }

    public void setFlip(FlipperPosition flipperPosition, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        this.flipperPosition = flipperPosition;
        periodic(0); // flush claw status
    }

    public FlipperPosition getFlipperPosition() {
        return this.flipperPosition;
    }

    public boolean isFlipperOnIntake() {
        return this.getFlipperPosition() == FlipperPosition.INTAKE;
    }

    public boolean leftClawRequestedToClose() {
        return closeLeftClaw;
    }

    public boolean rightClawRequestedToClose() {
        return closeRightClaw;
    }

    public void setAutoClosing(boolean autoClosing, ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;

        this.autoClosing = autoClosing;
    }

    public boolean leftClawInPosition() {
        return leftClaw.inPosition();
    }

    public boolean rightClawInPosition() {
        return rightClaw.inPosition();
    }

    public boolean flipInPosition() {
        return flip.inPosition();
    }

    public void setScoringAngle(double scoringOrHoldingClawAngle, ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;
        this.flipperPositions.put(FlipperPosition.SCORE, scoringOrHoldingClawAngle);
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        final Map<String, Object> debugMessages = new HashMap<>();
        debugMessages.put("flipper on intake", isFlipperOnIntake());
        return debugMessages;
    }
}
