package org.firstinspires.ftc.teamcode.Modules;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Utils.FixedAngleCameraProfile;
import org.firstinspires.ftc.teamcode.Utils.RawArilTagRecognitionCamera;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * TODO: add a param, the rotation of the pointing of the camera
 * */
public class FixedAngleArilTagCamera extends RobotModule {
    public static final double aprilTagMargin_CM = 16;

    private final RawArilTagRecognitionCamera camera;
    private final FixedAngleCameraProfile profile;

    private final WallTarget blueAllianceWall;
    private final WallTarget redAllianceWall;
    /** all the april tag targets for navigation */
    private final List<AprilTagTarget> aprilTagTargets;
    /** the wall that is currently in front of the Robot, null for unseen */
    private WallTarget wallInFront;

    static final HashMap<String, Object> debugMessages = new HashMap<>();

    public FixedAngleArilTagCamera(RawArilTagRecognitionCamera camera, FixedAngleCameraProfile cameraProfile) {
        super("AprilTagCamera");

        this.camera = camera;
        this.profile = cameraProfile;
        this.blueAllianceWall = new WallTarget(WallTarget.Name.BLUE_ALLIANCE_WALL);
        this.redAllianceWall = new WallTarget(WallTarget.Name.RED_ALLIANCE_WALL);

        this.aprilTagTargets = new ArrayList<>();
        Collections.addAll(this.aprilTagTargets, blueAllianceWall.aprilTagReferenceTargets);
        Collections.addAll(this.aprilTagTargets, redAllianceWall.aprilTagReferenceTargets);
    }

    @Override
    public void init() {
        camera.startRecognizing();
        reset();
    }

    @Override
    public void periodic(double dt) {
        camera.update();
        updateAprilTagTargets();
        updateWallInFront();
    }

    private void updateAprilTagTargets() {
        for (AprilTagTarget aprilTagTarget:aprilTagTargets) {
            RawArilTagRecognitionCamera.AprilTagTargetRaw rawTarget = camera.getRawAprilTagByID(aprilTagTarget.id);
            if (rawTarget == null)
                aprilTagTarget.setUnseen();
            else
                aprilTagTarget.updatePosition(processPosition(rawTarget));
        }
    }

    private void updateWallInFront() {
        this.wallInFront = null;
        if (redAllianceWall.isVisible())
            this.wallInFront = redAllianceWall;
        else if (blueAllianceWall.isVisible())
            this.wallInFront = blueAllianceWall;
    }

    @Override
    protected void onDestroy() {
        camera.stopRecognizing();
    }

    @Override
    public void reset() {
        this.wallInFront = null;
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        debugMessages.put("wall in front", wallInFront);
        if (wallInFront == null)
            debugMessages.put("wall in front", "null");
//        int tgt = 1;
//        for (AprilTagTarget aprilTagTarget:aprilTagTargets)
//            tmp.put("target "+tgt++, aprilTagTarget + "\n");
//
//        RawArilTagRecognitionCamera.AprilTagTargetRaw aprilTagTargetRaw = camera.getRawAprilTagByID(4);
//        if (aprilTagTargetRaw != null) tmp.put("id 4 rotation", Math.toDegrees(getTargetAngleRadianFromXPixel(aprilTagTargetRaw.x)));

        return debugMessages;
    }

    public List<AprilTagTarget> getArilTagTargets() {
        return aprilTagTargets;
    }

    public WallTarget getWallInFront() {
        return wallInFront;
    }

    private Vector2D processPosition(RawArilTagRecognitionCamera.AprilTagTargetRaw rawTarget) {
        final double yDistance = profile.getDistanceFromYPixel(rawTarget.y),
            xDistance = yDistance / Math.tan(profile.getTargetAngleRadianFromXPixel(rawTarget.x));
        return new Vector2D(new double[] {xDistance, yDistance});
    }

    public RawArilTagRecognitionCamera getRawAprilTagCamera() {
        return this.camera;
    }

    @Deprecated
    /**
     * process position using target and an adjacent target
     * measures distance using the margin
     * @Deprecated currently not in use
     * */
    private Vector2D processPosition(RawArilTagRecognitionCamera.AprilTagTargetRaw rawTarget, RawArilTagRecognitionCamera.AprilTagTargetRaw adjacentTarget) {
        final double
                adjacentDistanceAtMinDistance = this.profile.adjacentTargetRelativeDistanceRange[0],
                adjacentDistanceAtMaxDistance = this.profile.adjacentTargetRelativeDistanceRange[1],
                minDistance = this.profile.distanceRange[0],
                maxDistance = this.profile.distanceRange[1],
                distancePerAdjacentDistance = (maxDistance - minDistance) / (adjacentDistanceAtMaxDistance - adjacentDistanceAtMinDistance),
                // use the profile to process the target's distance
                xPixelDifference = rawTarget.x - adjacentTarget.x,
                yPixelDifference = rawTarget.y - adjacentTarget.y,
                adjacentTargetsPixelDistance = Math.sqrt(xPixelDifference * xPixelDifference + yPixelDifference * yPixelDifference),
                distance = minDistance + distancePerAdjacentDistance * (adjacentTargetsPixelDistance - adjacentDistanceAtMinDistance);
        return new Vector2D(
                profile.getTargetAngleRadianFromXPixel(rawTarget.x),
                distance);
    }

    /** stores an april tag target */
    public static final class AprilTagTarget {
        public final int id;
        public final Vector2D relativePositionToWall;
        /** null for unseen */
        private Vector2D relativePositionToRobot;
        /** system time when it was last seen */
        private long lastSeenTimeMillis;
        /** its position last time it was seen */
        private Vector2D lastSeenPositionToRobot;

        public AprilTagTarget(int id, Vector2D relativePositionToWall) {
            this.id = id;
            this.relativePositionToWall = relativePositionToWall;
            this.relativePositionToRobot = null;
            this.lastSeenPositionToRobot = new Vector2D(new double[] {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY});
            lastSeenTimeMillis = 0;
        }

        public void updatePosition(Vector2D relativePositionToRobot) {
            this.relativePositionToRobot = relativePositionToRobot;
            this.lastSeenPositionToRobot = relativePositionToRobot;
            this.lastSeenTimeMillis = System.currentTimeMillis();
        }

        public void setUnseen() {
            this.relativePositionToRobot = null;
        }

        /**
         * @return the relative position to robot, in centimeter and in reference to the robot
         * */
        public Vector2D getRelativePositionToRobot() {
            return this.relativePositionToRobot;
        }

        public boolean isVisible() {
            return this.getRelativePositionToRobot() != null;
        }

        @NonNull
        @Override
        public String toString() {
            if (!isVisible()) return "April tag (id:" + id + ") not seen";
            return "April tag (id:" + id + ") with relative position:" + getRelativePositionToRobot();
        }
    }

    public static final class WallTarget {
        public enum Name {
            BLUE_ALLIANCE_WALL,
            RED_ALLIANCE_WALL
        }
        public final Name name;

        /** a list of reference april tag target, and their positions to the wall */
        public final FixedAngleArilTagCamera.AprilTagTarget[] aprilTagReferenceTargets;

        WallTarget(Name wallName) {
            this.name = wallName;

            switch (name) {
                case RED_ALLIANCE_WALL: {
                    aprilTagReferenceTargets = new FixedAngleArilTagCamera.AprilTagTarget[] {
                            new FixedAngleArilTagCamera.AprilTagTarget(4, new Vector2D(new double[] {-aprilTagMargin_CM, 0})),
                            new FixedAngleArilTagCamera.AprilTagTarget(5, new Vector2D(new double[] {0, 0})),
                            new FixedAngleArilTagCamera.AprilTagTarget(6, new Vector2D(new double[] {aprilTagMargin_CM, 0}))
                    };
                    return;
                }
                case BLUE_ALLIANCE_WALL: {
                    aprilTagReferenceTargets = new FixedAngleArilTagCamera.AprilTagTarget[] {
                            new FixedAngleArilTagCamera.AprilTagTarget(1, new Vector2D(new double[] {-aprilTagMargin_CM, 0})),
                            new FixedAngleArilTagCamera.AprilTagTarget(2, new Vector2D(new double[] {0, 0})),
                            new FixedAngleArilTagCamera.AprilTagTarget(3, new Vector2D(new double[] {aprilTagMargin_CM, 0}))
                    };
                    return;
                }
                default:
                    throw new IllegalArgumentException("unknown wall name");
            }
        }

        /**
         * calculate, using available targets and the robot's rotation, the relative field position of the wall to the robot
         * @param robotRotation the rotation of the robot, in radian like geometry
         * @return relative field position to robot, in centimeter, null for currently unseen
         * */
        public Vector2D getRelativePositionToRobot(double robotRotation) {
            List<AprilTagTarget> visibleTargets = getVisibleTargets();
            if (visibleTargets.isEmpty()) return null;

            Vector2D relativePositionToRobot = new Vector2D();
            for (AprilTagTarget visibleReferenceTarget:visibleTargets) {
                final Vector2D tagRelativeFieldPositionToRobot = visibleReferenceTarget.getRelativePositionToRobot().multiplyBy(
                        new Rotation2D(robotRotation)),
                        wallPositionAccordingToCurrentReference = tagRelativeFieldPositionToRobot.addBy(
                                visibleReferenceTarget.relativePositionToWall.multiplyBy(-1));
                // debugMessages.put("position of tag " + visibleReferenceTarget.id, tagRelativeFieldPositionToRobot);
                relativePositionToRobot = relativePositionToRobot.addBy(wallPositionAccordingToCurrentReference);  // take average to positions determined by difference reference target
            }
            return relativePositionToRobot.multiplyBy(1.0f/visibleTargets.size());
        }

        /**
         * uses the references to measure the rotation of the wall
         * for example, if the robot's view is perpendicular to the wall's surface, it will be zero
         * if the robot's seeing the wall FROM THE RIGHT SIDE with a view of angle 45 degree, it should be -45deg
         * @return the relative rotation of the wall's back to the robot's robot, null for unable to determine
         * @deprecated the method is still been tested and not finished yet
         */
        @Deprecated
        public Rotation2D getRotationToRobot() { // TODO test this method
            List<AprilTagTarget> visibleTargets = getVisibleTargets();
            if (visibleTargets.size() < 2)
                return null;
            double rotationTotal = 0;
            for (int i = 0; i < visibleTargets.size()-1; i++) {
                final double yDifference = visibleTargets.get(i).getRelativePositionToRobot().getY() - visibleTargets.get(i+1).getRelativePositionToRobot().getY(),
                        xDifference = visibleTargets.get(i).getRelativePositionToRobot().getX() - visibleTargets.get(i+1).getRelativePositionToRobot().getX();
                rotationTotal += Math.atan2(yDifference, xDifference);
            }
            return new Rotation2D(rotationTotal / (visibleTargets.size()-1));
        }

        private List<AprilTagTarget> getVisibleTargets() {
            List<AprilTagTarget> visibleTargets = new ArrayList<>(1);
            for (AprilTagTarget aprilTagTarget:this.aprilTagReferenceTargets)
                if (aprilTagTarget.isVisible()) visibleTargets.add(aprilTagTarget);
            return visibleTargets;
        }

        /**
         * whether the target is currently visible
         * @return true for visible
         * */
        public boolean isVisible() {
            return !getVisibleTargets().isEmpty();
        }

        @NonNull
        @Override
        public String toString() {
            if (!isVisible()) return name.name() + " currently not seen";
            return "wall target " + name.name() + ", with relative (robot) position" + getRelativePositionToRobot(0);
        }
    }

}