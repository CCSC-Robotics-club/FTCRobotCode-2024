package org.firstinspires.ftc.teamcode.Utils;

public class SequentialCommandSegment {
    public final BezierCurveFeeder chassisMovementPathFeeder;
    public final Runnable beginning, periodic, ending;
    public final IsCompleteChecker isCompleteChecker;
    public final InitiateCondition initiateCondition;
    public final RotationFeeder startingRotationFeeder, endingRotationFeeder;
    public SequentialCommandSegment(BezierCurve pathSchedule, Runnable beginning, Runnable periodic, Runnable ending, IsCompleteChecker isCompleteChecker, double startingRotation, double endingRotation) {
        this(
                () -> true,
                pathSchedule, beginning, periodic, ending, isCompleteChecker, startingRotation, endingRotation
        );
    }

    public SequentialCommandSegment(InitiateCondition initiateCondition, BezierCurve pathSchedule, Runnable beginning, Runnable periodic, Runnable ending, IsCompleteChecker isCompleteChecker, double startingRotation, double endingRotation) {
        this(
                initiateCondition,
                () -> pathSchedule,
                beginning, periodic, ending,
                isCompleteChecker,
                () -> startingRotation,
                () -> endingRotation
        );
    }

    public SequentialCommandSegment(InitiateCondition initiateCondition,  BezierCurveFeeder pathFeeder, Runnable beginning, Runnable periodic, Runnable ending, IsCompleteChecker isCompleteChecker, RotationFeeder startingRotation, RotationFeeder endingRotation) {
        this.chassisMovementPathFeeder = pathFeeder;

        this.beginning = beginning;
        this.periodic = periodic;
        this.ending = ending;

        this.isCompleteChecker = isCompleteChecker;
        this.initiateCondition = initiateCondition;

        this.startingRotationFeeder = startingRotation;
        this.endingRotationFeeder = endingRotation;
    }

    public static double getCurrentRotationWithLERP(double startingRotation, double endingRotation, double t) {
        if (t<0) t=0;
        else if (t>1) t=1;
        return AngleUtils.simplifyAngle(startingRotation + AngleUtils.getActualDifference(startingRotation, endingRotation)*t);
    }

    public double getStartingRotation() {
        return startingRotationFeeder.getRotation();
    }

    public double getEndingRotation() {
        return endingRotationFeeder.getRotation();
    }

    public double getMaxAngularVelocity() {
        return Math.abs(AngleUtils.getActualDifference(getStartingRotation(),  getEndingRotation()));
    }

    public BezierCurve getChassisMovementPath() {
        return chassisMovementPathFeeder.getBezierCurve();
    }

    public interface IsCompleteChecker {
        /** check whether this checkpoint */
        boolean isComplete();
    }

    public interface InitiateCondition {
        boolean initiateOrSkip();
    }

    public interface BezierCurveFeeder {
        BezierCurve getBezierCurve();
    }
    public interface RotationFeeder {
        double getRotation();
    }
}
