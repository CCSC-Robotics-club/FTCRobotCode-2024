package org.firstinspires.ftc.teamcode.Modules;


import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.FixedAngleArilTagCamera;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.AngleUtils;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.EncoderMotorMechanism;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.EnhancedPIDController;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.EnhancedPIDController2D;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.SimpleFeedForwardController;
import org.firstinspires.ftc.teamcode.Utils.ModulesCommanderMarker;
import org.firstinspires.ftc.teamcode.Utils.PositionEstimator;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Transformation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import static org.firstinspires.ftc.teamcode.RobotConfig.VisualNavigationConfigs;
import static org.firstinspires.ftc.teamcode.RobotConfig.ChassisConfigs;

import java.util.HashMap;
import java.util.Map;

public class Chassis extends RobotModule {
    private final EncoderMotorMechanism frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel;
    private final SimpleFeedForwardController frontLeftController, frontRightController, backLeftController, backRightController;
    private final PositionEstimator positionEstimator;
    private final FixedAngleArilTagCamera aprilTagCamera;

    private final EnhancedPIDController rotationController;
    private final EnhancedPIDController2D translationalControllerEncoder;

    private final FixedAngleArilTagCamera.WallTarget.Name allianceWallName;

    private final Map<String, Object> debugMessages;

    private ChassisTranslationalTask translationalTask;
    private ChassisRotationalTask rotationalTask;
    private OrientationMode orientationMode;

    /** null for never seen */
    private Vector2D wallAbsoluteEncoderPositionField;
    private Vector2D wallEncoderPositionForFaceToWallControlOnly;
    /** -1 for never seen */
    private long wallLastSeenTimeMillis;

    private boolean lowSpeedModeEnabled = false;

    public Chassis(
            EncoderMotorMechanism frontLeftWheel, EncoderMotorMechanism frontRightWheel, EncoderMotorMechanism backLeftWheel, EncoderMotorMechanism backRightWheel,
            PositionEstimator positionEstimator, FixedAngleArilTagCamera aprilTagCamera, FixedAngleArilTagCamera.WallTarget.Name allianceWall
    ) {
        super("chassis module", 100);

        this.frontLeftWheel = frontLeftWheel;
        this.frontRightWheel = frontRightWheel;
        this.backLeftWheel = backLeftWheel;
        this.backRightWheel = backRightWheel;
        this.positionEstimator = positionEstimator;
        this.aprilTagCamera = aprilTagCamera;
        this.debugMessages = new HashMap<>(1);
        this.allianceWallName = allianceWall;

        this.rotationController = new EnhancedPIDController(ChassisConfigs.chassisRotationControllerProfile);
        this.translationalControllerEncoder = new EnhancedPIDController2D(ChassisConfigs.encoderTranslationalControllerProfileX, ChassisConfigs.encoderTranslationalControllerProfileY);

        this.frontLeftController = new SimpleFeedForwardController(ChassisConfigs.wheelPowerLookUpTable);
        this.frontRightController = new SimpleFeedForwardController(ChassisConfigs.wheelPowerLookUpTable);
        this.backLeftController = new SimpleFeedForwardController(ChassisConfigs.wheelPowerLookUpTable);
        this.backRightController = new SimpleFeedForwardController(ChassisConfigs.wheelPowerLookUpTable);

        frontLeftWheel.setController(frontLeftController);
        frontRightWheel.setController(frontRightController);
        backLeftWheel.setController(backLeftController);
        backRightWheel.setController(backRightController);
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    public void periodic(double dt) {
        Vector2D calculatedTranslationalSpeed = calculateTranslationalSpeedWithProperMethod(translationalTask);
        double calculatedRotationalSpeed = calculateRotationalMotorSpeedWithProperMethod(rotationalTask, dt);

        if (translationalTask.taskType == ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY && this.orientationMode == OrientationMode.FIELD_ORIENTATED)
            calculatedTranslationalSpeed = calculatedTranslationalSpeed.multiplyBy(
                    new Rotation2D(getYaw()).getReversal());

        final long t = System.currentTimeMillis();
        driveMecanumWheels(calculatedTranslationalSpeed, calculatedRotationalSpeed);
        debugMessages.put("drive wheels time", System.currentTimeMillis() - t);
    }

    private void driveMecanumWheels(Vector2D translationalMotion, double rotationalMotion) {
        if (lowSpeedModeEnabled)
            driveMecanumWheels(translationalMotion, rotationalMotion, ChassisConfigs.lowSpeedModeMaximumMotorSpeedConstrain);
        else
            driveMecanumWheels(translationalMotion, rotationalMotion, ChassisConfigs.ordinaryModeMaximumMotorSpeedConstrain);
    }

    private void driveMecanumWheels(Vector2D translationalMotion, double rotationalMotion, double powerConstrain) {
        final double xMotion = translationalMotion.getX(),
                yMotion = translationalMotion.getY();

        double frontLeftWheelMotorPower = yMotion - rotationalMotion + xMotion,
                backLeftWheelMotorPower = yMotion - rotationalMotion - xMotion,
                frontRightWheelMotorPower = yMotion + rotationalMotion - xMotion,
                backRightWheelMotorPower = yMotion + rotationalMotion + xMotion,
                greatestPower = Math.max(
                        Math.max(Math.abs(frontLeftWheelMotorPower), Math.abs(frontRightWheelMotorPower)),
                        Math.max(Math.abs(backLeftWheelMotorPower), Math.abs(backRightWheelMotorPower)));
        if (greatestPower > powerConstrain) {
            frontLeftWheelMotorPower *= (powerConstrain / greatestPower);
            frontRightWheelMotorPower *= (powerConstrain / greatestPower);
            backLeftWheelMotorPower *= (powerConstrain / greatestPower);
            backRightWheelMotorPower *= (powerConstrain / greatestPower);
        }

        frontLeftController.setDesiredSpeed(frontLeftWheelMotorPower);
        frontRightController.setDesiredSpeed(frontRightWheelMotorPower);
        backLeftController.setDesiredSpeed(backLeftWheelMotorPower);
        backRightController.setDesiredSpeed(backRightWheelMotorPower);
    }

    private Vector2D calculateTranslationalSpeedWithProperMethod(ChassisTranslationalTask task) {
        if (task==null)
            return new Vector2D();
        switch (task.taskType) {
            case DRIVE_TO_POSITION_ENCODER:
                return processEncoderDriveToPositionControl(task.translationalValue);
            case DRIVE_TO_POSITION_VISUAL:
                return processVisualNavigationControl();
            case SET_VELOCITY:
                return translationalTask.translationalValue.multiplyBy(lowSpeedModeEnabled ? ChassisConfigs.lowSpeedModeMaximumMotorSpeedConstrain:1);
            default:
                return new Vector2D();
        }
    }

    private double calculateRotationalMotorSpeedWithProperMethod(ChassisRotationalTask task, double dt) {
        debugMessages.put("rotation mode", task.taskType);
        switch (task.taskType) {
            case SET_ROTATIONAL_SPEED: {
                return task.rotationalValue;
            }
            case GO_TO_ROTATION: {
                rotationController.startNewTask(new EnhancedPIDController.Task(
                        EnhancedPIDController.Task.TaskType.GO_TO_POSITION,
                        task.rotationalValue
                ));
                return rotationController.getMotorPower(getYaw(), getYawVelocity(), dt);
            }
            case FACE_NAVIGATION_REFERENCES: {
//                return calculateRotationalMotorSpeedWithProperMethod(
//                        new ChassisRotationalTask(ChassisRotationalTask.ChassisRotationalTaskType.GO_TO_ROTATION, getWallDirectionWithVisualNavigation()),
//                        dt);
                return calculateRotationalMotorSpeedWithProperMethod(
                        new ChassisRotationalTask(ChassisRotationalTask.ChassisRotationalTaskType.GO_TO_ROTATION,
                                getEncoderTargetRotation(this.wallEncoderPositionForFaceToWallControlOnly) - Math.PI / 2
                        ),
                        dt
                );
            }
            default: return 0;
        }
    }


    private double getEncoderTargetRotation(Vector2D target) {
        Vector2D difference = target.addBy(getChassisEncoderPosition().multiplyBy(-1));
        return difference.getHeading();
    }

    private void initiateFaceWallTargetRotationTask(ChassisRotationalTask task) {
        if (!isVisualNavigationAvailable()
                || this.rotationalTask.taskType == ChassisRotationalTask.ChassisRotationalTaskType.FACE_NAVIGATION_REFERENCES)
            return;
        updateVisualTargetAndEncoderReference();
        this.wallEncoderPositionForFaceToWallControlOnly = wallAbsoluteEncoderPositionField;
        this.rotationalTask = task;
    }


    /**
     * @return the correction power, in reference to the robot
     * */
    @Deprecated
    private Vector2D processEncoderDriveToPositionControl_old(Vector2D desiredEncoderPosition) {
        this.translationalControllerEncoder.startNewTask(
                new EnhancedPIDController2D.Task2D(EnhancedPIDController.Task.TaskType.GO_TO_POSITION, desiredEncoderPosition));

        final Vector2D correctionPowerToField = translationalControllerEncoder.getCorrectionPower(this.positionEstimator.getCurrentPosition(), this.positionEstimator.getCurrentVelocity(OrientationMode.FIELD_ORIENTATED)),
            correctionPowerToRobot = correctionPowerToField.multiplyBy(
                new Rotation2D(positionEstimator.getRotation())
                        .getReversal());
        debugMessages.put("pid control desired position", desiredEncoderPosition);
        // debugMessages.put("correction power to field", correctionPowerToField);
        final Transformation2D motorPowerRate = new Transformation2D(
                new Vector2D(new double[] {ChassisConfigs.xPowerRate, 0}),
                new Vector2D(new double[] {0, ChassisConfigs.yPowerRate}));

        return correctionPowerToRobot.multiplyBy(motorPowerRate);
    }

    private Vector2D processEncoderDriveToPositionControl(Vector2D desiredEncoderPosition) {
        final Vector2D positionDifferenceField = desiredEncoderPosition.addBy(getChassisEncoderPosition().multiplyBy(-1)),
            positionDifferenceRobot  = positionDifferenceField.multiplyBy(new Rotation2D(getYaw()).getReversal());
        this.translationalControllerEncoder.startNewTask(
                new EnhancedPIDController2D.Task2D(EnhancedPIDController.Task.TaskType.GO_TO_POSITION, positionDifferenceRobot));

        final Vector2D correctionPowerToRobot = translationalControllerEncoder.getCorrectionPower(new Vector2D(), this.positionEstimator.getCurrentVelocity(OrientationMode.ROBOT_ORIENTATED));
        final Transformation2D motorPowerRate = new Transformation2D(
                new Vector2D(new double[] {ChassisConfigs.xPowerRate, 0}),
                new Vector2D(new double[] {0, ChassisConfigs.yPowerRate}));

        return correctionPowerToRobot.multiplyBy(motorPowerRate);
    }

    /**
     * get
     * @return the correction motor speed of the robot, in percent output
     */
    private Vector2D processVisualNavigationControl() {
        /* translation:
         * the encoder is reliable and sensitive during a short period of time
         * therefore, it should be used to make up for the low fps of husky cam
         * we use encoders to measure the change in position during two adjacent updates from the husky cam
         * we update the husky cam 5 times a second only
         *
         * also, since the delay of the camera is too high, when the robot moves too quickly it will be very inaccurate
         * so we must update the wall position only if velocity is lower than a value
         * and restrict the robot's correction power to below a certain value
         */
        if (System.currentTimeMillis() - wallLastSeenTimeMillis > 1000 / VisualNavigationConfigs.visualModuleUpdateRate
                && isVisualNavigationReliable()) // if it is time to update and there is reliable visual navigation
            updateVisualTargetAndEncoderReference();

        final Vector2D wallEncoderPosition = wallAbsoluteEncoderPositionField, // use the data obtained last time to determine where the wall is matter of ENCODER position
                targetedEncoderPosition = wallEncoderPosition.addBy(translationalTask.translationalValue); // our targeted translational value is in reference to the wall
        debugMessages.put("wall pos", wallEncoderPosition);
        debugMessages.put("navigation reliable", isVisualNavigationReliable());

        return processEncoderDriveToPositionControl(targetedEncoderPosition);
    }

    /**
     * for driving bezier curves when trying to drive to april tag
     * so the system does not update the visual target encoder position
     * @param newTranslationalTargetVisual the new translational target, in related to the wall target
     * @param operator the module or service that called to this command
     * */
    public void updateDesiredTranslationInVisualNavigation(Vector2D newTranslationalTargetVisual, ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;
        if (this.translationalTask.taskType != ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_VISUAL)
            return;
        debugMessages.put("updated visual task to", newTranslationalTargetVisual);
        this.translationalTask = new ChassisTranslationalTask(ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_VISUAL, newTranslationalTargetVisual);
    }

    /** forces periodic to the camera */
    public void forceUpdateCamera(ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;
        aprilTagCamera.updateCamera();
    }

    public void forceUpdateEncoders(ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;
        positionEstimator.forceUpdate();
    }

    public void forceUpdateWheels(ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;
        frontLeftWheel.updateWithController();
        frontRightWheel.updateWithController();
        backLeftWheel.updateWithController();
        backRightWheel.updateWithController();
    }

    private void updateVisualTargetAndEncoderReference() {
        if (!isVisualNavigationAvailable()) throw new IllegalStateException("attempt to update visual target when no visual navigation reference tag available");
        final Vector2D wallRelativeFieldPositionToRobotWhenLastSeen = aprilTagCamera.getWallInFront().getRelativePositionToRobot(getYaw()),
                encoderPositionReferenceWhenWallLastSeen = getChassisEncoderPosition(),
                newWallPositionEncoder =  wallRelativeFieldPositionToRobotWhenLastSeen.addBy(encoderPositionReferenceWhenWallLastSeen);

        if (this.wallAbsoluteEncoderPositionField != null &&
                Vector2D.displacementToTarget(this.wallAbsoluteEncoderPositionField, newWallPositionEncoder).getMagnitude() < VisualNavigationConfigs.errorTolerance)
            return;

        this.wallAbsoluteEncoderPositionField = newWallPositionEncoder;
        this.wallLastSeenTimeMillis = System.currentTimeMillis();
    }

    public boolean isVisualNavigationAvailable() {
        final FixedAngleArilTagCamera.WallTarget wallTarget = aprilTagCamera.getWallInFront();
        return wallTarget != null && wallTarget.isVisible() && wallTarget.name == allianceWallName;
    }

    private boolean isVisualNavigationReliable() {
        if (!isVisualNavigationAvailable())
            return false;
        return this.positionEstimator.getCurrentVelocity(OrientationMode.ROBOT_ORIENTATED).getMagnitude() < VisualNavigationConfigs.visualModuleResultReliableVelocityMax
                && Math.abs(getYawVelocity()) < VisualNavigationConfigs.visualModuleResultReliableAngularSpeedMax;
    }


    /**
     * gets the relative position of the robot to the alliance's wall using fresh visual camera results
     * @return the relative field position of the robot to the wall, in cm. null for unseen
     * */
    public Vector2D getRelativeFieldPositionToWall() {
        final FixedAngleArilTagCamera.WallTarget wallInFront = aprilTagCamera.getWallInFront();
        if (wallInFront == null || wallInFront.getRelativePositionToRobot(getYaw()) == null)
            return null;
        return wallInFront.getRelativePositionToRobot(getYaw()).multiplyBy(-1);
    }

    private void initiateVisualNavigationTask(ChassisTranslationalTask newVisualTranslationalTask) {
        if (this.translationalTask.taskType == ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_VISUAL
                && newVisualTranslationalTask.translationalValue.equals(this.translationalTask.translationalValue))
            return;
        if (!isVisualNavigationAvailable())
            return;
        updateVisualTargetAndEncoderReference();
        this.translationalTask = newVisualTranslationalTask;
    }

    @Override
    protected void onDestroy() {

    }

    @Override
    public void reset() {
        setOrientationMode(OrientationMode.ROBOT_ORIENTATED, null);
        this.translationalTask = new ChassisTranslationalTask(ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY, new Vector2D());
        this.rotationalTask = new ChassisRotationalTask(ChassisRotationalTask.ChassisRotationalTaskType.SET_ROTATIONAL_SPEED, 0);
        resetYaw(null);

        this.wallAbsoluteEncoderPositionField = null;

        this.lowSpeedModeEnabled = false;

        driveMecanumWheels(new Vector2D(), 0);
    }

    public void setOrientationMode(OrientationMode orientationMode, ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;

        this.orientationMode = orientationMode;
    }

    public OrientationMode getOrientationMode() {
        return this.orientationMode;
    }

    public void setTranslationalTask(ChassisTranslationalTask translationalTask, ModulesCommanderMarker operator) {
        debugMessages.put("set trans task", translationalTask.translationalValue);
        debugMessages.put("operator", operator);
        debugMessages.put("is owner", isOwner(operator));
        if (!isOwner(operator))
            return;

        switch (translationalTask.taskType) {
            case SET_VELOCITY:
            case DRIVE_TO_POSITION_ENCODER: {
                this.translationalTask = translationalTask;
                break;
            }
            case DRIVE_TO_POSITION_VISUAL: {
                this.initiateVisualNavigationTask(translationalTask);
                break;
            }
        }

        debugMessages.put("chassis task set result", this.translationalTask.translationalValue);
    }

    public void setTranslationalTaskToBreak(ModulesCommanderMarker operator) {
        setTranslationalTask(new ChassisTranslationalTask(ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER, getChassisEncoderPosition()) , operator);
    }

    public void setRotationalTask(ChassisRotationalTask rotationalTask, ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;

        switch (rotationalTask.taskType) {
            case SET_ROTATIONAL_SPEED:
            case GO_TO_ROTATION: {
                this.rotationalTask = rotationalTask;
                break;
            }
            case FACE_NAVIGATION_REFERENCES: {
                initiateFaceWallTargetRotationTask(rotationalTask);
                break;
            }
        }
    }

    public void setLowSpeedModeEnabled(boolean lowSpeedModeEnabled) {
        this.lowSpeedModeEnabled = lowSpeedModeEnabled;
    }

    public boolean isLowSpeedModeEnabled() {
        return this.lowSpeedModeEnabled;
    }

    public void resetYaw(ModulesCommanderMarker operator) {
        if (!isOwner(operator))
            return;
        this.positionEstimator.calibrateRotation();
    }

    public Vector2D getChassisEncoderPosition() {
        return positionEstimator.getCurrentPosition();
    }

    static final double zeroJudge = 0.01;
    public boolean isCurrentTranslationalTaskRoughlyComplete() {
        return isCurrentTranslationalTaskComplete(ChassisConfigs.errorAsTaskRoughlyFinished);
    }
    public boolean isCurrentTranslationalTaskComplete() {
        return isCurrentTranslationalTaskComplete(ChassisConfigs.errorAsTaskFinishedCM) && positionEstimator.getCurrentVelocity(OrientationMode.ROBOT_ORIENTATED).getMagnitude() < ChassisConfigs.chassisSpeedAsRobotStoppedCMPerSec;
    }
    private boolean isCurrentTranslationalTaskComplete(double errorTolerance) {
        switch (translationalTask.taskType) {
            case SET_VELOCITY:
                return translationalTask.translationalValue.getMagnitude() < zeroJudge; // set velocity is a continuous command, it is only finished if the pilot idles the controller
            case DRIVE_TO_POSITION_ENCODER:
                return isCloseEnough(getChassisEncoderPosition(), translationalTask.translationalValue, errorTolerance);
            case DRIVE_TO_POSITION_VISUAL:
                return isCloseEnough(getChassisEncoderPosition(), wallAbsoluteEncoderPositionField.addBy(translationalTask.translationalValue), errorTolerance);
            default:
                throw new IllegalArgumentException("unknown translational task" + translationalTask.taskType.name());
        }
    }

    public boolean isCurrentRotationalTaskRoughlyComplete() {
        return isCurrentRotationalTaskComplete(ChassisConfigs.chassisRotationErrorAsRoughlyFinished);
    }

    public boolean isCurrentRotationalTaskComplete() {
        return isCurrentRotationalTaskComplete(ChassisConfigs.chassisRotationErrorAsFinished)
                && Math.abs(positionEstimator.getAngularVelocity()) < ChassisConfigs.chassisRotationSpeedAsStopped;
    }

    private boolean isCurrentRotationalTaskComplete(double errorTolerance) {
        switch (rotationalTask.taskType) {
            case SET_ROTATIONAL_SPEED:
                return Math.abs(rotationalTask.rotationalValue) < zeroJudge;
            case GO_TO_ROTATION:
                return Math.abs(AngleUtils.getActualDifference(positionEstimator.getRotation(), rotationalTask.rotationalValue))
                        < errorTolerance;
            default:
                throw new IllegalArgumentException("unknown rotational task" + rotationalTask.taskType.name());
        }
    }

    public ChassisTranslationalTask getCurrentTranslationalTask() {
        return this.translationalTask;
    }

    public ChassisRotationalTask getCurrentRotationalTask() {
        return this.rotationalTask;
    }

    private static boolean isCloseEnough(Vector2D currentPosition, Vector2D desiredPosition, double errorTolerance) {
        return Vector2D.displacementToTarget(currentPosition, desiredPosition).getMagnitude() < errorTolerance;
    }

                                               /**
     * get the reading of the chassis imu
     * @return in radian and counter-clockwise is positive
     */
    public double getYaw() {
        return this.positionEstimator.getRotation();
    }

    public Rotation2D getRotation() {
        return new Rotation2D(getYaw());
    }

    public void setCurrentYaw(double yaw) {
        positionEstimator.setRotation(yaw);
    }

    public double getYawVelocity() {
        return this.positionEstimator.getAngularVelocity();
    }

    public static class ChassisTranslationalTask {
        private final ChassisTranslationalTaskType taskType;
        private final Vector2D translationalValue;
        private final double initiateTimeNano;
        public ChassisTranslationalTask(ChassisTranslationalTaskType taskType, Vector2D translationalValue) {
            this.taskType = taskType;
            this.translationalValue = translationalValue;
            this.initiateTimeNano = System.nanoTime();
        }

        /**
         * get the task timer value
         * @return the amount of time, in seconds, from the initiation of the task to now
         */
        public double getTaskTime() {
            return (System.nanoTime() - initiateTimeNano) / 1_000_000_000.0;
        }

        public Vector2D getTranslationalValue() {
            return translationalValue;
        }

        public ChassisTranslationalTaskType getTaskType() {
            return taskType;
        }

        public enum ChassisTranslationalTaskType {
            DRIVE_TO_POSITION_VISUAL,
            DRIVE_TO_POSITION_ENCODER,
            SET_VELOCITY
        }
    }

    public static class ChassisRotationalTask {
        private final ChassisRotationalTaskType taskType;
        /**
         * in radian
         */
        private final double rotationalValue;
        private final double initiateTimeNano;
        public ChassisRotationalTask(ChassisRotationalTaskType taskType, double rotationalValue) {
            this.taskType = taskType;
            this.rotationalValue = rotationalValue;
            this.initiateTimeNano = System.nanoTime();
        }

        /**
         * get the task timer value
         * @return the amount of time, in seconds, from the initiation of the task to now
         */
        public double getTaskTime() {
            return (System.nanoTime() - initiateTimeNano) / 1_000_000_000.0;
        }

        public double getRotationalValue() {
            return rotationalValue;
        }

        public ChassisRotationalTaskType getTaskType() {
            return taskType;
        }

        public enum ChassisRotationalTaskType {
            GO_TO_ROTATION,
            SET_ROTATIONAL_SPEED,
            FACE_NAVIGATION_REFERENCES
        }
    }

    @Override
    public Map<String, Object> getDebugMessages() {
//        debugMessages.put("chassis rotation task type", rotationalTask.taskType);
//        debugMessages.put("chassis rotation task value", Math.toDegrees(rotationalTask.rotationalValue));
//        debugMessages.put("chassis yaw", Math.toDegrees(getYaw()));
//        debugMessages.put("yaw vel", Math.toDegrees(getYawVelocity()));
        debugMessages.put("chassis update freq", super.getUpdateCountPerSecond());
        return debugMessages;
    }

    public enum OrientationMode {
        FIELD_ORIENTATED,
        ROBOT_ORIENTATED
    }
}