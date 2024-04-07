package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.ProfiledServo;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

import static org.firstinspires.ftc.teamcode.RobotConfig.FlippableDualClawConfigs;

import java.util.HashMap;
import java.util.Map;

public class FlippableDualClaw extends RobotModule {
    private final ProfiledServo flip, leftClaw, rightClaw;

    private boolean closeLeftClaw, closeRightClaw, flipperOnIntake;
    public FlippableDualClaw(Servo flip, Servo leftClaw, Servo rightClaw) {
        super("claw");
        /* TODO: make servo speed in robot config */
        this.flip = new ProfiledServo(flip, 2);
        this.leftClaw = new ProfiledServo(leftClaw, 2);
        this.rightClaw = new ProfiledServo(rightClaw,2);
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    protected void periodic(double dt) {
        leftClaw.setDesiredPosition(closeLeftClaw ? FlippableDualClawConfigs.leftClawClosePosition : FlippableDualClawConfigs.leftClawOpenPosition);
        rightClaw.setDesiredPosition(closeRightClaw ? FlippableDualClawConfigs.rightClawClosedPosition : FlippableDualClawConfigs.rightClawOpenPosition);
        flip.setDesiredPosition(flipperOnIntake ? FlippableDualClawConfigs.flipperIntakePosition : FlippableDualClawConfigs.flipperNormalPosition);

        leftClaw.update(dt);
        rightClaw.update(dt);
        flip.update(dt);
    }

    @Override
    protected void onDestroy() {

    }

    @Override
    public void reset() {
        closeLeftClaw = closeRightClaw = flipperOnIntake = false;

        flip.setDesiredPosition(FlippableDualClawConfigs.flipperNormalPosition);
        flip.update(10);
        leftClaw.setDesiredPosition(FlippableDualClawConfigs.leftClawClosePosition);
        leftClaw.update(10);
        rightClaw.setDesiredPosition(FlippableDualClawConfigs.rightClawClosedPosition);
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

    public void setFlip(boolean flipOnIntakePosition, RobotService operatorService) {
        if (!isOwner(operatorService))
            return;
        this.flipperOnIntake = flipOnIntakePosition;
        periodic(0); // flush claw status
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

    @Override
    public Map<String, Object> getDebugMessages() {
        final Map<String, Object> debugMessages = new HashMap<>();
        debugMessages.put("flipper on intake", flipperOnIntake);
        return debugMessages;
    }
}
