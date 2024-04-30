package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.MotorThreaded;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ProfiledServo;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.RobotService;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.SimpleSensor;

import static org.firstinspires.ftc.teamcode.RobotConfig.FlippableDualClawConfigs;

import java.util.HashMap;
import java.util.Map;

public class FlippableDualClaw extends RobotModule {
    private final ProfiledServo flip, leftClaw, rightClaw;
    private final SimpleSensor detectorLeft, detectorRight;
    private final MotorThreaded leftIndicatorLight, rightIndicatorLight;

    private boolean closeLeftClaw, closeRightClaw, flipperOnIntake, autoClosing;
    private double scoringOrHoldingClawAngle;
    public FlippableDualClaw(ProfiledServo flip, ProfiledServo leftClaw, ProfiledServo rightClaw, SimpleSensor detectorLeft, SimpleSensor detectorRight, MotorThreaded leftIndicatorLight, MotorThreaded rightIndicatorLight) {
        super("claw");
        this.flip = flip;
        this.leftClaw = leftClaw;
        this.rightClaw = rightClaw;

        this.detectorLeft = detectorLeft;
        this.detectorRight = detectorRight;
        this.leftIndicatorLight = leftIndicatorLight;
        this.rightIndicatorLight = rightIndicatorLight;
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    protected void periodic(double dt) {
        final boolean leftClawDetected = detectorLeft.getSensorReading() > FlippableDualClawConfigs.colorDetectorThreshold,
                rightClawDetected = detectorRight.getSensorReading() > FlippableDualClawConfigs.colorDetectorThreshold;

        closeLeftClaw |= autoClosing && leftClawDetected;
        closeRightClaw |= autoClosing && rightClawDetected;

        /* flip automatically */
        if (autoClosing && leftClawDetected && rightClawDetected && leftClaw.inPosition() && rightClaw.inPosition())
            this.flipperOnIntake = false;

        leftClaw.setDesiredPosition(closeLeftClaw ? FlippableDualClawConfigs.leftClawClosePosition : FlippableDualClawConfigs.leftClawOpenPosition);
        rightClaw.setDesiredPosition(closeRightClaw ? FlippableDualClawConfigs.rightClawClosedPosition : FlippableDualClawConfigs.rightClawOpenPosition);
        flip.setDesiredPosition(
                flipperOnIntake ? FlippableDualClawConfigs.flipperIntakePosition
                : this.scoringOrHoldingClawAngle
        );

        leftClaw.update(dt);
        rightClaw.update(dt);
        flip.update(dt);

        final double blink = 0.5+0.5*Math.sin(System.currentTimeMillis() / 1000.0f * 2);
        leftIndicatorLight.setPower(autoClosing ? (leftClawDetected ? 1 : blink) : 0);
        rightIndicatorLight.setPower(autoClosing ? (rightClawDetected ? 1 : blink) : 0);
    }

    @Override
    protected void onDestroy() {

    }

    @Override
    public void reset() {
        closeLeftClaw = closeRightClaw = flipperOnIntake = autoClosing = false;

        leftIndicatorLight.setPower(0);
        rightIndicatorLight.setPower(0);

        flip.setDesiredPosition(FlippableDualClawConfigs.flipperHoldPosition);
        flip.update(10);
        leftClaw.setDesiredPosition(FlippableDualClawConfigs.leftClawClosePosition);
        leftClaw.update(10);
        rightClaw.setDesiredPosition(FlippableDualClawConfigs.rightClawClosedPosition);
        rightClaw.update(10);

        scoringOrHoldingClawAngle = FlippableDualClawConfigs.flipperHoldPosition;
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

    public void setFlip(boolean flipOnIntakePosition, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        this.flipperOnIntake = flipOnIntakePosition;
        periodic(0); // flush claw status
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
        this.scoringOrHoldingClawAngle = scoringOrHoldingClawAngle;
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        final Map<String, Object> debugMessages = new HashMap<>();
        debugMessages.put("flipper on intake", flipperOnIntake);
        return debugMessages;
    }
}
