package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Claw;
import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;

import static org.firstinspires.ftc.teamcode.RobotConfig.ArmConfigs;

public class ExtendableClaw extends RobotModule {
    private final Claw claw;
    private final Servo extendControllerServo;

    private enum Status {
        STANDBY,
        HOLD_PIXEL,
        PLACE_TO_BOARD,
        RELEASE
    }
    private Status status;
    private double timer=0;
    public ExtendableClaw(Claw claw, Servo extendControllerServo) {
        super("Extendable-Claw", 20);
        this.claw = claw;
        this.extendControllerServo = extendControllerServo;
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    protected void periodic(double dt) {
        timer += dt;
        switch (status) {
            case RELEASE: {
                claw.open();
                extendControllerServo.setPosition(ArmConfigs.servoValueOrigin);
                return;
            }
        }
    }

    @Override
    protected void onDestroy() {

    }

    @Override
    public void reset() {
        this.status = Status.STANDBY;
        claw.open();
        extendControllerServo.setPosition(ArmConfigs.servoValueOrigin);
    }

    public void holdPixel(ModulesCommanderMarker operator) {
        if (!isOwner(operator)) return;
        if (status == Status.STANDBY)
            status = Status.HOLD_PIXEL;
    }

    public void placePixelToBoard(ModulesCommanderMarker operator) {
        if (!isOwner(operator)) return;
        if (status == Status.HOLD_PIXEL)
            status = Status.PLACE_TO_BOARD;
        this.timer = 0;
    }
}
