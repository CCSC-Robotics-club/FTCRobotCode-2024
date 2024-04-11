package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.firstinspires.ftc.teamcode.AutoStages.CameraAutoCalibration;
import org.firstinspires.ftc.teamcode.AutoStages.CameraAutoCalibrationVerticalOnly;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;

@Autonomous(name = "<Auto> [Camera Calibration]")
public class CameraCalibrationAutoProgram extends AutoMain {
    public CameraCalibrationAutoProgram() {
//        super(new CameraAutoCalibration(
//                22,
//                new Vector2D(new double[] {0, -5}),
//                2,
//                new Rotation2D(Math.toRadians(90)) // facing front
//        ));

        super(new CameraAutoCalibrationVerticalOnly(
                22,
                new Vector2D(new double[] {0, -3}),
                2
        ));
    }
}
