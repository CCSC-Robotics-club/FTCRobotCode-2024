package org.firstinspires.ftc.teamcode.Services;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Utils.RobotService;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.Vector2D;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.RobotConfig.ChassisConfigs;

/**
 * runs a sequence of command segment
 * the auto stage of our robot is basically running the modules in this service, simulating a pilot's commands
 */
public class AutoProgramRunner extends RobotService {
    private final List<SequentialCommandSegment> commandSegments;
    private int currentSegment;
    private final Chassis robotChassis;
    private double currentSegmentTime;
    private double currentSegmentChassisPathTimeScale; // slow the time down when smaller than 1 (=1/ETA)
    private final Map<String, Object> debugMessages = new HashMap<>();
    private boolean errorOccured = false;

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
        robotChassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY,
                new Vector2D()), this);
        robotChassis.setRotationalTask(new Chassis.ChassisRotationalTask(
                Chassis.ChassisRotationalTask.ChassisRotationalTaskType.SET_ROTATIONAL_SPEED,
                0), this);

        if (commandSegments.size() - currentSegment == 1)
            return;
        initiateSegment(currentSegment);
        if (commandSegments.get(currentSegment).chassisMovementPath == null) return;
        double distanceBetweenCurrentEndToNextStart = Vector2D.displacementToTarget(
                commandSegments.get(currentSegment).chassisMovementPath.getPositionWithLERP(1),
                commandSegments.get(++currentSegment).chassisMovementPath.getPositionWithLERP(0))
                .getMagnitude();
        if (distanceBetweenCurrentEndToNextStart > 5) // TODO check it when initialization
            throw new IllegalArgumentException("current segment (id:" + currentSegment + ")'s starting point does match the ending point of the last segment with deviation " + distanceBetweenCurrentEndToNextStart);

    }
    private void initiateSegment(int segmentID) {
        this.currentSegmentTime = 0;
        this.currentSegmentChassisPathTimeScale = getTimeScaleWithMaximumVelocityAndAcceleration();
        try {
            this.commandSegments.get(segmentID).beginning.run();
        } catch (Exception e) {
            errorOccured = true;
        }

        if (commandSegments.get(segmentID).chassisMovementPath != null) robotChassis.gainOwnerShip(this);
    }
    private double getTimeScaleWithMaximumVelocityAndAcceleration() {
        final double maxVel = this.commandSegments.get(currentSegment).chassisMovementPath.maximumSpeed;
        final double maxAcc = this.commandSegments.get(currentSegment).chassisMovementPath.maximumAcceleration;
        final double maxAngularVel = this.commandSegments.get(currentSegment).maxAngularVelocity;

        return Math.min(Math.min(ChassisConfigs.autoStageMaxAcceleration / maxAcc, ChassisConfigs.autoStageMaxVelocity/ maxVel), ChassisConfigs.autoStageMaxAngularVelocity / maxAngularVel);
    }

    public boolean isAutoStageComplete() {
        if (errorOccured) return true;
        return this.commandSegments.size() - this.currentSegment == 1
                && this.isCurrentSegmentComplete()
                && robotChassis.isCurrentTranslationalTaskComplete()
                && robotChassis.isCurrentRotationalTaskComplete();
    }

    public boolean isCurrentSegmentComplete() {
        SequentialCommandSegment currentSegment = this.commandSegments.get(this.currentSegment);
        final double ETA = (1.0f/currentSegmentChassisPathTimeScale);
        debugMessages.put("currentSegmentTime >= ETA", currentSegmentTime >= ETA);
        debugMessages.put(" currentSegment.isCompleteChecker.isComplete()",  currentSegment.isCompleteChecker.isComplete());
        return currentSegmentTime >= ETA
                && currentSegment.isCompleteChecker.isComplete();
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        debugMessages.put("segment id", currentSegment);
        debugMessages.put("is auto complete", isAutoStageComplete());
        return debugMessages;
    }
}
