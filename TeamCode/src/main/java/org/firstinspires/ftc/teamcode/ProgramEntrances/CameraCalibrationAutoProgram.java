package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.firstinspires.ftc.teamcode.AutoStages.CameraAutoCalibration;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;

@Autonomous(name = "<Auto> [Camera Calibration]")
public class CameraCalibrationAutoProgram extends AutoMain {
    public CameraCalibrationAutoProgram() {
        super(new CameraAutoCalibration(
                22,
                new Vector2D(new double[] {0, -5}),
                2
        ));
    }
}
