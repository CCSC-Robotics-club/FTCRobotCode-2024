package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.stream.StreamSupport;

public class EnhancedPIDController2D {
    private final EnhancedPIDController xController, yController;
    private long previousTimeMillis;

    public EnhancedPIDController2D(EnhancedPIDController.PIDProfile config) {
        this(config, config);
    }

    public EnhancedPIDController2D(EnhancedPIDController.PIDProfile xConfig, EnhancedPIDController.PIDProfile yConfig) {
        this.xController = new EnhancedPIDController(xConfig);
        this.yController = new EnhancedPIDController(yConfig);
        this.previousTimeMillis = System.currentTimeMillis();
    }

    public void startNewTask(Task2D task) {
        xController.startNewTask(task.xTask);
        yController.startNewTask(task.yTask);
    }

    public Vector2D getCorrectionPower(Vector2D currentPosition) {
        double dt = (System.currentTimeMillis() - previousTimeMillis) / 1000.0f;
        double xCorrectionPower = this.xController.getMotorPower(currentPosition.getX(), dt);
        double yCorrectionPower = this.yController.getMotorPower(currentPosition.getY(), dt);
        this.previousTimeMillis = System.currentTimeMillis();
        return new Vector2D(new double[]{xCorrectionPower, yCorrectionPower});
    }

    public Vector2D getCorrectionPower(Vector2D currentPosition, Vector2D currentVelocity) {
        double dt = (System.currentTimeMillis() - previousTimeMillis) / 1000.0f;
        double xCorrectionPower = this.xController.getMotorPower(currentPosition.getX(), currentVelocity.getX(), dt);
        double yCorrectionPower = this.yController.getMotorPower(currentPosition.getY(), currentVelocity.getY(), dt);
        this.previousTimeMillis = System.currentTimeMillis();
        return new Vector2D(new double[]{xCorrectionPower, yCorrectionPower});
    }

    public static final class Task2D {
        public final EnhancedPIDController.Task xTask, yTask;
        public Task2D(EnhancedPIDController.Task.TaskType taskType, Vector2D value) {
            this.xTask = new EnhancedPIDController.Task(taskType, value.getX());
            this.yTask = new EnhancedPIDController.Task(taskType, value.getY());
        }
    }
}
