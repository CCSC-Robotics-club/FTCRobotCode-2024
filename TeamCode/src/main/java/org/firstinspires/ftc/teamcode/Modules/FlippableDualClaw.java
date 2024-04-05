package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

import static org.firstinspires.ftc.teamcode.RobotConfig.FlippableDualClawConfigs;

public class FlippableDualClaw extends RobotModule {
    private final Servo flip, leftClaw, rightClaw;

    private boolean leftClawClosed, rightClawClosed, flipperOnIntake;
    public FlippableDualClaw(Servo flip, Servo leftClaw, Servo rightClaw) {
        super("claw");
        this.flip = flip;
        this.leftClaw = leftClaw;
        this.rightClaw = rightClaw;
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    protected void periodic(double dt) {
        leftClaw.setPosition(leftClawClosed ? FlippableDualClawConfigs.leftClawClosePosition : FlippableDualClawConfigs.leftClawOpenPosition);
        rightClaw.setPosition(rightClawClosed ? FlippableDualClawConfigs.rightClawClosedPosition : FlippableDualClawConfigs.rightClawOpenPosition);
        flip.setPosition(flipperOnIntake ? FlippableDualClawConfigs.flipperIntakePosition : FlippableDualClawConfigs.flipperScoringPosition);
    }

    @Override
    protected void onDestroy() {

    }

    @Override
    public void reset() {
        leftClawClosed = rightClawClosed = flipperOnIntake = false;
    }

    public void setLeftClawClosed(boolean closed, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        this.leftClawClosed = closed;
    }

    public void setRightClawClosed(boolean closed, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        this.rightClawClosed = closed;
    }

    public void setFlip(boolean flipOnIntakePosition, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        this.flipperOnIntake = flipOnIntakePosition;
    }

    public boolean isFlipOnIntake() {
        return flipperOnIntake;
    }

    public boolean isLeftClawClosed() {
        return leftClawClosed;
    }

    public boolean isRightClawClosed() {
        return rightClawClosed;
    }
}
