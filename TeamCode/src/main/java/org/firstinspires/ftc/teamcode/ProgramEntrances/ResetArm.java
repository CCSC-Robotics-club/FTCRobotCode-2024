package org.firstinspires.ftc.teamcode.ProgramEntrances;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Arm-Reset")
public class ResetArm extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "limit");
        final DcMotor armMotor = hardwareMap.get(DcMotor.class, "arm");

        waitForStart();

        final double powerRate = 1;
        final int startingPos = armMotor.getCurrentPosition();
        while (opModeIsActive() && !isStopRequested()) {
            double power = gamepad1.left_stick_y * powerRate;
            if (Math.abs(power) < 0.05)
                power = 0;

            armMotor.setPower(power);

            telemetry.addData("limit switch pressed", limitSwitch.getValue());
            telemetry.addData("arm encoder reading", armMotor.getCurrentPosition() - startingPos);
            telemetry.update();

            sleep(50);
        }
    }
}
