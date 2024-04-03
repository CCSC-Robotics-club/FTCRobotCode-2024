package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;

/**  */
public interface PositionEstimator {
    /**
     * sets the current position to be a given value
     * @param currentPosition in meters
     * */
    void setCurrentPosition(Vector2D currentPosition);
    /** set the current position as origin */
    default void calibratePosition() {
        setCurrentPosition(new Vector2D());
    }

    /**
     * get the current velocity of the robot
     * @param orientationMode the mode of orientation
     * @return the velocity of the robot, orientated to the selected mode, and in meters/second
     * */
    Vector2D getCurrentVelocity(Chassis.OrientationMode orientationMode);

    /**
     * get the current rotation of the robot
     * @return in radian, counter-clockwise is positive
     * */
    double getRotation();


    /** get the current rotational velocity of the robot
     * @return in radian/seconds, counter-clockwise is positive
     * */
    double getAngularVelocity();

    /** set the current rotation as zero reference */
    default void calibrateRotation() {
        setRotation(0);
    }

    /** set the current rotation as a given reading */
    void setRotation(double givenRotation);

    /**
     * get the current position of the robot
     * @return orientated to the field, and in meters
     * */
    Vector2D getCurrentPosition();

    void forceUpdate();
}
