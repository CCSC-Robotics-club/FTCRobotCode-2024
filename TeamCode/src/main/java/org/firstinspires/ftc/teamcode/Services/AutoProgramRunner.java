package org.firstinspires.ftc.teamcode.Services;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Utils.RobotService;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.Vector2D;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * runs a sequence of command segment
 * the auto stage of our robot is basically running the modules in this service, simulating a pilot's commands
 */
public class AutoProgramRunner extends RobotService {
    private static final int autoStageRunnerUpdateRate = 20;
    private final List<SequentialCommandSegment> commandSegments;
    private int currentSegment;
    private final Chassis robotChassis;
    private double currentSegmentTime;
    private double currentSegmentChassisPathTimeScale; // slow the time down when smaller than 1 (=1/ETA)
    private boolean segmentEndingComplete;
    private Map<String, Object> debugMessages = new HashMap<>();

    public AutoProgramRunner(List<SequentialCommandSegment> commandSegments, Chassis chassis) {
        this.commandSegments = commandSegments;
        this.robotChassis = chassis;
    }

    @Override
    public void init() {
        this.reset();
    }

    @Override
    public void periodic(double dt) {
        currentSegmentTime += dt;

        final SequentialCommandSegment currentCommandSegment = commandSegments.get(currentSegment);
        final double t = currentSegmentTime * currentSegmentChassisPathTimeScale;
        if (commandSegments.get(currentSegment).chassisMovementPath != null)
            robotChassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                            Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                            currentCommandSegment.chassisMovementPath.getPositionWithLERP(t)),
                    this);
        debugMessages.put("time(scaled)", t);
        debugMessages.put("time(raw)", currentSegmentTime);
        debugMessages.put("dt(s)", dt);
        debugMessages.put("ETA",1.0f/currentSegmentChassisPathTimeScale);
        debugMessages.put("chassis desired position", currentCommandSegment.chassisMovementPath.getPositionWithLERP(t));
        robotChassis.setRotationalTask(new Chassis.ChassisRotationalTask(
                        Chassis.ChassisRotationalTask.ChassisRotationalTaskType.GO_TO_ROTATION,
                        currentCommandSegment.getCurrentRotationWithLERP(t)),
                this);
        currentCommandSegment.periodic.run();

        if (isCurrentSegmentComplete())
            nextSegment();
    }

    @Override
    public void onDestroy() {

    }

    @Override
    public void reset() {
        this.currentSegment = 0;
        robotChassis.gainOwnerShip(this);
        initiateSegment(0);
    }

    private void nextSegment() {
        this.commandSegments.get(currentSegment).ending.run();
        this.segmentEndingComplete = true;
        robotChassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY,
                new Vector2D()), this);
        robotChassis.setRotationalTask(new Chassis.ChassisRotationalTask(
                Chassis.ChassisRotationalTask.ChassisRotationalTaskType.SET_ROTATIONAL_SPEED,
                0), this);

        if (commandSegments.size() - currentSegment == 1)
            return;
        double distanceBetweenCurrentEndToNextStart = Vector2D.displacementToTarget(
                commandSegments.get(currentSegment).chassisMovementPath.getPositionWithLERP(1),
                commandSegments.get(++currentSegment).chassisMovementPath.getPositionWithLERP(0))
                .getMagnitude();
        if (distanceBetweenCurrentEndToNextStart > 5)
            throw new IllegalArgumentException("current segment (id:" + currentSegment + ")'s starting point does match the ending point of the last segment with deviation " + distanceBetweenCurrentEndToNextStart);
        initiateSegment(currentSegment);
    }
    private void initiateSegment(int segmentID) {
        this.currentSegmentTime = 0;
        this.segmentEndingComplete = false;
        this.currentSegmentChassisPathTimeScale = getTimeScaleWithMaximumVelocityAndAcceleration();
        this.commandSegments.get(segmentID).beginning.run();
        if (commandSegments.get(segmentID).chassisMovementPath != null) robotChassis.gainOwnerShip(this);
    }
    private double getTimeScaleWithMaximumVelocityAndAcceleration() {
        /* TODO gain from Robot config */
        final double maxVelAllowed = 500; // cm/s
        final double maxAccAllowed = 500; // cm*s^-2
        final double maxAngularVelAllowed = Math.PI * 2;

        final double maxVel = this.commandSegments.get(currentSegment).chassisMovementPath.maximumSpeed;
        final double maxAcc = this.commandSegments.get(currentSegment).chassisMovementPath.maximumAcceleration;
        final double maxAngularVel = this.commandSegments.get(currentSegment).maxAngularVelocity;

        return Math.min(Math.min(maxAccAllowed / maxAcc, maxVelAllowed / maxVel), maxAngularVelAllowed / maxAngularVel);
    }

    public boolean isAutoStageComplete() {
        return this.commandSegments.size() - this.currentSegment == 1 && this.isCurrentSegmentComplete();
    }

    public boolean isCurrentSegmentComplete() {
        SequentialCommandSegment currentSegment = this.commandSegments.get(this.currentSegment);
        return currentSegmentTime >= (1.0f/currentSegmentChassisPathTimeScale)
                && currentSegment.isCompleteChecker.isComplete()
                && robotChassis.isCurrentTranslationalTaskComplete()
                && robotChassis.isCurrentRotationalTaskComplete()
                && segmentEndingComplete;
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        return super.getDebugMessages();
    }
}
