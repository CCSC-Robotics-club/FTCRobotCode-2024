package org.firstinspires.ftc.teamcode.Utils;

public class BezierCurvePathSchedule {
    private final ChassisProfile chassisProfile;
    private final BezierCurve[] segments;
    /** the time when each segment begins */
    private final double[] segmentTimeStamps;
    /** the timescale of each segment, the higher it is, THE FASTER THE ROBOT GOES IN THE VIEW OF THE PHYSICAL WORLD and THE TIME NEEDED FOR A SEGMENT IS SMALLER
     * (the system scales the time down or up so that the robot motions at the speed that satisfies the maximum acc and vel)
     * timeNeeded = 1/timeScaleRatio
     * */
    private final double[] timeScales;

    public double ETA;
    public BezierCurvePathSchedule(ChassisProfile chassisProfile, BezierCurve[] bezierCurves) {
        this.chassisProfile = chassisProfile;
        this.segments = bezierCurves;
        this.segmentTimeStamps = new double[bezierCurves.length];
        this.segmentTimeStamps[0] = 0; // starting time is 0
        this.timeScales = new double[bezierCurves.length];
        scheduleCheckPoints();
    }

    private void scheduleCheckPoints() {
        for (int segmentID = 0; segmentID < segments.length; segmentID++)
            scheduleCheckPoints(segmentID);
    }

    private void scheduleCheckPoints(int segmentID) {
        BezierCurve segment = segments[segmentID];

        double maximumAcc = segment.maximumAcceleration;
        double maximumVel = segment.maximumSpeed;

        /* scale the time down by a factor, so it does not exceed the maximum acc or vel */
        double timeScaleRatio = Math.min(this.chassisProfile.maximumAcceleration / maximumAcc, this.chassisProfile.maximumSpeed / maximumVel);
        double timeNeeded = 1/timeScaleRatio;

        this.timeScales[segmentID] = timeScaleRatio;

        double timeWhenCompleted = this.segmentTimeStamps[segmentID] + timeNeeded;
        if (segmentID == segments.length-1)  // if this is the last segment
            ETA = timeWhenCompleted;
        else
            this.segmentTimeStamps[segmentID+1] = timeWhenCompleted;
    }

    public Vector2D getPosition(double t) {
        if (t <= 0)
            return segments[0].getPositionWithLERP(0); // starting point

        for (int segmentID = 0; segmentID < this.segments.length-1; segmentID++) { // check all the segments, from start to end
            if (t > segmentTimeStamps[segmentID+1]) continue; // if t is not in this segment, continue searching
            return getPosition(t, segmentID);
        }
        // if it is not find inside all previous segments
        return getPosition(t, segments.length-1); // find it in the last one
    }

    public Vector2D getPosition(double t, int segmentID) {
        t -= this.segmentTimeStamps[segmentID];
        t *= this.timeScales[segmentID];
        BezierCurve segment = segments[segmentID];
        System.out.println("position:" + segment.getPositionWithLERP(t));
        return segment.getPositionWithLERP(t);
    }

    public boolean isComplete(double t) {
        return t > ETA;
    }

    public static class ChassisProfile {
        public final double maximumSpeed;
        public final double maximumAcceleration;
        public ChassisProfile(double maximumSpeed, double maximumAcceleration) {
            this.maximumSpeed = maximumSpeed;
            this.maximumAcceleration = maximumAcceleration;
        }
    }
}
