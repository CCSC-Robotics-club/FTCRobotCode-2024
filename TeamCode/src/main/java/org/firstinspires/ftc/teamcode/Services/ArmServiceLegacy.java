package org.firstinspires.ftc.teamcode.Services;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.ArmLegacy;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

import static org.firstinspires.ftc.teamcode.RobotConfig.ArmConfigsLegacy;

@Deprecated
public class ArmServiceLegacy extends RobotService {
    private final ArmLegacy arm;
    private final Gamepad copilotGamepad;

    private ArmLegacy.ArmCommand currentArmCommand = null;
    public ArmServiceLegacy(ArmLegacy arm, Gamepad copilotGamepad) {
        this.arm = arm;
        this.copilotGamepad = copilotGamepad;
    }
    @Override
    public void init() {
        reset();
    }

    @Override
    public void periodic(double dt) {
        if (Math.abs(copilotGamepad.left_stick_y) > 0.05) {
            this.currentArmCommand = new ArmLegacy.ArmCommand(ArmLegacy.ArmCommand.ArmCommandType.SET_MOTOR_POWER, -copilotGamepad.left_stick_y);
        } else if (copilotGamepad.y) {
            this.currentArmCommand = new ArmLegacy.ArmCommand(ArmLegacy.ArmCommand.ArmCommandType.SET_POSITION, ArmConfigsLegacy.highPos);
        } else if (copilotGamepad.b) {
            this.currentArmCommand = new ArmLegacy.ArmCommand(ArmLegacy.ArmCommand.ArmCommandType.SET_POSITION, ArmConfigsLegacy.midPos);
        } else if (copilotGamepad.a) {
            this.currentArmCommand = new ArmLegacy.ArmCommand(ArmLegacy.ArmCommand.ArmCommandType.SET_POSITION, ArmConfigsLegacy.feedPos);
        }else if (ArmConfigsLegacy.armCommandWhenNoInput != null){
            this.currentArmCommand = ArmConfigsLegacy.armCommandWhenNoInput;
        }

        if (copilotGamepad.left_bumper) arm.placePixel(this);
        else if (copilotGamepad.left_trigger > RobotConfig.ControlConfigs.triggerThreshold) arm.holdPixel(this);

        arm.setArmCommand(currentArmCommand, this);
    }

    @Override
    public void onDestroy() {

    }

    @Override
    public void reset() {
        this.arm.reset();
        this.arm.gainOwnerShip(this);
        this.currentArmCommand = ArmConfigsLegacy.armCommandWhenNoInput;
    }
}
