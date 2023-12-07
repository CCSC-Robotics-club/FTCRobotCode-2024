package org.firstinspires.ftc.teamcode.Services;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Utils.BezierCurve;
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
    private boolean errorOccurred = false;

    public AutoProgramRunner(List<SequentialCommandSegment> commandSegments, Chassis chassis) {
        this.commandSegments = commandSegments;
        this.robotChassis = chassis;
    }

    @Override
    public void init() {
//        /* check if there is any jump in the starting and ending point */
//        for (int currentSegment = 0; currentSegment < commandSegments.size()-1; currentSegment++) {
//            if (commandSegments.get(currentSegment).getChassisMovementPath() == null || commandSegments.get(currentSegment+1).getChassisMovementPath() == null)
//                continue;
//            final double distanceBetweenCurrentEndToNextStart = Vector2D.displacementToTarget(
//                            commandSegments.get(currentSegment).getChassisMovementPath().getPositionWithLERP(1),
//                            commandSegments.get(currentSegment+1).getChassisMovementPath().getPositionWithLERP(0))
//                    .getMagnitude();
//            if (distanceBetweenCurrentEndToNextStart > 10)
//                throw new IllegalArgumentException("current segment (id:" + currentSegment + ")'s starting point does match the ending point of the last segment with deviation " + distanceBetweenCurrentEndToNextStart);
//        }
        this.reset();
    }

    @Override
    public void periodic(double dt) {
        if (isAutoStageComplete())
            return;
        currentSegmentTime += dt;

        final SequentialCommandSegment currentCommandSegment = commandSegments.get(currentSegment);
        final double t = currentSegmentTime * currentSegmentChassisPathTimeScale;
        if (commandSegments.get(currentSegment).getChassisMovementPath() != null)
            robotChassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                            Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                            currentPath.getPositionWithLERP(t)),
                    this);
        debugMessages.put("time(scaled)", t);
        debugMessages.put("time(raw)", currentSegmentTime);
        debugMessages.put("dt(s)", dt);
        debugMessages.put("ETA",1.0f/currentSegmentChassisPathTimeScale);
        if (currentPath!=null) debugMessages.put("chassis desired position", currentPath.getPositionWithLERP(t));
        robotChassis.setRotationalTask(new Chassis.ChassisRotationalTask(
                        Chassis.ChassisRotationalTask.ChassisRotationalTaskType.GO_TO_ROTATION,
                        SequentialCommandSegment.getCurrentRotationWithLERP(currentSegmentStartingRotation, currentSegmentEndingRotation, t)),
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

        if (currentSegment+1 >= commandSegments.size()) {
            robotChassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY, new Vector2D()), this);
            robotChassis.setRotationalTask(new Chassis.ChassisRotationalTask(Chassis.ChassisRotationalTask.ChassisRotationalTaskType.SET_ROTATIONAL_SPEED, 0), this);
        }
        else initiateSegment(++currentSegment);
    }

    private double currentSegmentStartingRotation = 0, currentSegmentEndingRotation = 0, currentSegmentMaxAngularVelocity = 0;
    private BezierCurve currentPath = null;
    private void initiateSegment(int segmentID) {
        final SequentialCommandSegment segment = this.commandSegments.get(segmentID);
        if (!segment.initiateCondition.initiateOrSkip()) {
            nextSegment(); // skip this segment
            return;
        }

        this.currentSegmentTime = 0;
        segment.beginning.run();

        this.currentSegmentStartingRotation = segment.getStartingRotation();
        this.currentSegmentEndingRotation = segment.getEndingRotation();
        this.currentSegmentMaxAngularVelocity = segment.getMaxAngularVelocity();
        this.currentPath = segment.getChassisMovementPath();

        if (segment.getChassisMovementPath() == null) return;
        robotChassis.gainOwnerShip(this);
        this.currentSegmentChassisPathTimeScale = getTimeScaleWithMaximumVelocityAndAcceleration();
    }
    private double getTimeScaleWithMaximumVelocityAndAcceleration() {
        final double maxVel = currentPath.maximumSpeed;
        final double maxAcc = currentPath.maximumAcceleration;
        final double maxAngularVel = currentSegmentMaxAngularVelocity;

        return Math.min(Math.min(ChassisConfigs.autoStageMaxAcceleration / maxAcc, ChassisConfigs.autoStageMaxVelocity/ maxVel), ChassisConfigs.autoStageMaxAngularVelocity / maxAngularVel);
    }

    public boolean isAutoStageComplete() {
        if (errorOccurred) return true;
        return this.commandSegments.size() - this.currentSegment == 1
                && this.isCurrentSegmentComplete();
    }

    public boolean isCurrentSegmentComplete() {
        SequentialCommandSegment currentSegment = this.commandSegments.get(this.currentSegment);
        final double ETA = (1.0f/currentSegmentChassisPathTimeScale);
        return currentSegmentTime >= ETA
                && currentSegment.isCompleteChecker.isComplete();
//                && robotChassis.isCurrentTranslationalTaskRoughlyComplete()
//                && robotChassis.isCurrentRotationalTaskComplete();
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        debugMessages.put("segment id", currentSegment);
        debugMessages.put("is auto complete", isAutoStageComplete());
        return debugMessages;
    }
}
