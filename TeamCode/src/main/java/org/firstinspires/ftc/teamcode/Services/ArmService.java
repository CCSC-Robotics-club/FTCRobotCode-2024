package org.firstinspires.ftc.teamcode.Services;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Arm;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

import static org.firstinspires.ftc.teamcode.RobotConfig.ArmConfigs;

public class ArmService extends RobotService {
    private final Arm arm;
    private final Gamepad copilotGamepad;

    private Arm.ArmCommand currentArmCommand = null;
    public ArmService(Arm arm, Gamepad copilotGamepad) {
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
            this.currentArmCommand = new Arm.ArmCommand(Arm.ArmCommand.ArmCommandType.SET_MOTOR_POWER, -copilotGamepad.left_stick_y);
        } else if (copilotGamepad.y) {
            this.currentArmCommand = new Arm.ArmCommand(Arm.ArmCommand.ArmCommandType.SET_POSITION, ArmConfigs.highPos);
        } else if (copilotGamepad.b) {
            this.currentArmCommand = new Arm.ArmCommand(Arm.ArmCommand.ArmCommandType.SET_POSITION, ArmConfigs.midPos);
        } else if (copilotGamepad.a) {
            this.currentArmCommand = new Arm.ArmCommand(Arm.ArmCommand.ArmCommandType.SET_POSITION, ArmConfigs.feedPos);
        }else if (ArmConfigs.armCommandWhenNoInput != null){
            this.currentArmCommand = ArmConfigs.armCommandWhenNoInput;
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
        this.currentArmCommand = ArmConfigs.armCommandWhenNoInput;
    }
}
