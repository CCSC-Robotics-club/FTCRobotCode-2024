package org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils;

import java.util.List;

public interface RawArilTagRecognitionCamera {
    void startRecognizing();
    void stopRecognizing();
    void update();
    List<AprilTagTargetRaw> getRawArilTagTargets();

    /**
     * gets an april tag target by its id
     * @param id the id of the desired april tag
     * @return the april tag raw target if seen, or null if unseen
     * */
    default AprilTagTargetRaw getRawAprilTagByID(int id) {
        try {
            for (AprilTagTargetRaw targetRaw : getRawArilTagTargets())
                if (targetRaw.id == id) return targetRaw;
        } catch (NullPointerException ignored) {}
        return null;
    }

    final class AprilTagTargetRaw {
        public final int id;
        /** in reference to the center of the camera view */
        public final double x, y;
        public AprilTagTargetRaw(int id, double x, double y) {
            this.id = id;
            this.x = x;
            this.y = y;
        }
    }
}
