package org.firstinspires.ftc.teamcode.Services;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.BezierCurveSchedule;
import org.firstinspires.ftc.teamcode.Utils.RobotService;
import org.firstinspires.ftc.teamcode.Utils.SequentialCommandSegment;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * runs a sequence of command segment
 * the auto stage of our robot is basically running the modules in this service, simulating a pilot's commands
 */
public class AutoProgramRunner extends RobotService {
    private List<SequentialCommandSegment> commandSegments;

    private final Chassis robotChassis;
    private int currentSegmentID;
    private SequentialCommandSegment.StaticSequentialCommandSegment currentCommandSegment;
    private BezierCurveSchedule currentPathSchedule;
    private double currentSegmentRotationScheduleETA, rotationT, translationalScaledT;

    public AutoProgramRunner(Chassis chassis) {
        this.robotChassis = chassis;
    }

    @Override
    public void init() {
        this.reset();
    }

    @Override
    public void periodic(double dt) {
        if (currentSegmentID == -1)
            initiateSegment(0);

        robotChassis.setOrientationMode(Chassis.OrientationMode.FIELD_ORIENTATED, this);


        if (currentPathSchedule != null) {
            translationalScaledT = currentPathSchedule.nextCheckPoint(dt * currentCommandSegment.timeScale);
            final Vector2D inAdvanceSpaceWithoutConstrain =  currentPathSchedule.getVelocityWithLERP().multiplyBy(RobotConfig.ChassisConfigs.autoStageInAdvanceTime),
                    distanceLeft = Vector2D.displacementToTarget(currentPathSchedule.getPositionWithLERP(), currentPathSchedule.getPositionWithLERP(1)),
                    inAdvanceSpaceWithConstrain = new Vector2D(inAdvanceSpaceWithoutConstrain.getHeading(), Math.min(inAdvanceSpaceWithoutConstrain.getMagnitude(), distanceLeft.getMagnitude()));
            robotChassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                    Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                            currentPathSchedule.getPositionWithLERP().addBy(
                                    (
                                            currentSegmentID == commandSegments.size()-1
                                                    || commandSegments.get(currentSegmentID+1).chassisMovementPathFeeder.getBezierCurve() == null
                                    ) ? inAdvanceSpaceWithConstrain : inAdvanceSpaceWithoutConstrain)),
                    this);

        }
        if (currentSegmentRotationScheduleETA != -1) {
            rotationT += dt / currentSegmentRotationScheduleETA;
            double rotationTSyncedToTranslationT = rotationT;

            if (currentPathSchedule != null)
                rotationTSyncedToTranslationT = Math.min(currentPathSchedule.getT(), rotationTSyncedToTranslationT);
            robotChassis.setRotationalTask(new Chassis.ChassisRotationalTask(
                            Chassis.ChassisRotationalTask.ChassisRotationalTaskType.GO_TO_ROTATION,
                            currentCommandSegment.getCurrentRotationWithLERP(rotationTSyncedToTranslationT)),
                    this);
        }
        currentCommandSegment.periodic.run();

        if (isCurrentSegmentComplete()) {
            this.commandSegments.get(currentSegmentID).ending.run();
            nextSegment();
        }
    }

    @Override
    public void onDestroy() {

    }

    @Override
    public void reset() {
        this.currentSegmentID = -1;
        robotChassis.gainOwnerShip(this);

        commandSegments = new ArrayList<>();
    }

    private void nextSegment() {
        if (currentSegmentID+1 < commandSegments.size())
            initiateSegment(currentSegmentID+1);
    }

    private void initiateSegment(int segmentID) {
        this.currentSegmentID = segmentID;
        currentCommandSegment = this.commandSegments.get(segmentID).embodyCurrentCommandSegment();

        if (!currentCommandSegment.initiateCondition.initiateOrSkip()) {
            System.out.println("skipping segment: " + segmentID);
            nextSegment();
        }

        currentCommandSegment.beginning.run();

        final boolean rotationSpecified = currentCommandSegment.startingRotation != null && currentCommandSegment.endingRotation != null;
        this.currentSegmentRotationScheduleETA = rotationSpecified ?
                BezierCurveSchedule.getTimeNeededToFinishRotationalSchedule(currentCommandSegment.startingRotation.getRadian(), currentCommandSegment.endingRotation.getRadian())
                : -1;
        rotationT = 0;

        if (currentCommandSegment.chassisMovementPath == null) {
            this.currentPathSchedule = null;
            return;
        }
        this.currentPathSchedule = BezierCurveSchedule.generateTranslationalSchedule(currentCommandSegment.chassisMovementPath);
        robotChassis.gainOwnerShip(this);
    }

    public void setCommandSegments(List<SequentialCommandSegment> commandSegments) {
        this.commandSegments = commandSegments;
    }

    public boolean isAutoStageComplete() {
        return this.commandSegments.size() - this.currentSegmentID == 1
                && this.isCurrentSegmentComplete();
    }

    public boolean isCurrentSegmentComplete() {
        SequentialCommandSegment currentSegment = this.commandSegments.get(this.currentSegmentID);
        if (currentSegment.endEarlyChecker.isComplete())
            return true;
        final boolean translationalMovementFinished = currentPathSchedule == null || currentPathSchedule.isCurrentPathFinished();
        final boolean rotationalMovementFinished = currentSegmentRotationScheduleETA == -1 || rotationT >= 1;

//        if (!translationalMovementFinished)
//            System.out.println("<-- Auto Program Runner | waiting for path to finish -->");
//        else if (!rotationalMovementFinished)
//            System.out.println("<-- Auto Program Runner | waiting for rotation schedule to finish, t: " + rotationT + " -->");
//        else if (!currentSegment.isCompleteChecker.isComplete())
//            System.out.println("<-- Auto Program Runner | waiting for is complete checker to confirm complete -->");
        return translationalMovementFinished
                && rotationalMovementFinished
                && currentSegment.isCompleteChecker.isComplete();
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        final Map<String, Object> debugMessages = new HashMap<>();
        debugMessages.put("auto segment ID", currentSegmentID);
        if (currentPathSchedule == null)
            return debugMessages;
        debugMessages.put("auto translational scaled T", translationalScaledT);
        debugMessages.put("auto position (x)", currentPathSchedule.getPositionWithLERP().getX());
        debugMessages.put("auto position (y)", currentPathSchedule.getPositionWithLERP().getY());
        return debugMessages;
    }
}
