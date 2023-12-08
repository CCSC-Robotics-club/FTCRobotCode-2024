package org.firstinspires.ftc.teamcode.ProgramEntrances;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Modules.Arm;
import org.firstinspires.ftc.teamcode.RobotConfig;

@TeleOp(name="Arm-Reset")
public class ResetArm extends LinearOpMode {
    @Override
    public void runOpMode() {
        final TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "limit");
        final DcMotor armMotor1 = hardwareMap.get(DcMotor.class, "arm1"), armMotor2 = hardwareMap.get(DcMotor.class, "arm2");

        waitForStart();

        final int startingPos = armMotor1.getCurrentPosition();
        while (opModeIsActive() && !isStopRequested()) {
            double power = -gamepad1.left_stick_y;
            if (Math.abs(power) < 0.05) power = 0;

            armMotor1.setPower(RobotConfig.ArmConfigs.armMotor1Reversed ? -power:power);
            armMotor2.setPower(RobotConfig.ArmConfigs.armMotor2Reversed ? -power:power);

            telemetry.addData("limit switch pressed", limitSwitch.getValue());
            telemetry.addData("arm encoder reading", armMotor1.getCurrentPosition() - startingPos);
            telemetry.update();

            sleep(50);
        }
    }
}
