package org.firstinspires.ftc.teamcode.Services;

import static org.firstinspires.ftc.teamcode.RobotConfig.ChassisConfigs.timeToStartDecelerate;
import static org.firstinspires.ftc.teamcode.RobotConfig.ChassisConfigs.targetDistanceAtMaxDesiredSpeed;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.RobotService;
import org.firstinspires.ftc.teamcode.Utils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.DriverGamePad;

import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

public class PilotChassisService extends RobotService {
    private final Chassis chassis;
    private final DriverGamePad driverController;
    public final DistanceSensor distanceSensor;
    private final boolean independentEncodersAvailable, visualNavigationSupported;
    private double rotationWhenStickPressed;
    private Vector2D currentDesiredPosition;
    /** time since last translational command sent by pilot */
    private double pilotLastTranslationalActionTime;

    public enum ControlMode {
        MANUAL,
        MANUAL_FIELD_ORIENTATED,
        ENCODER_ASSISTED_FIELD_ORIENTATED
    }
    private ControlMode controlMode;

    private Map<String, Object> debugMessages = new HashMap<>(1);
    public PilotChassisService(Chassis chassis, DriverGamePad driverController, DistanceSensor distanceSensor, boolean independentEncodersAvailable, boolean visualNavigationSupported) {
        this.chassis = chassis;
        this.driverController = driverController;
        this.distanceSensor = distanceSensor;
        this.independentEncodersAvailable = independentEncodersAvailable;
        this.visualNavigationSupported = visualNavigationSupported && independentEncodersAvailable && distanceSensor != null;
    }
    @Override
    public void init() {
        this.reset();
    }

    @Override
    public void periodic(double dt) {
        /* <--translation--> */
        final double zeroJudge = 0.001;
        Vector2D pilotTranslationalCommand = driverController.getTranslationStickVector();
        Chassis.ChassisTranslationalTask translationalTaskByPilotStickControl = new Chassis.ChassisTranslationalTask(
                Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY,
                pilotTranslationalCommand
        );

        if (pilotTranslationalCommand.getMagnitude() > zeroJudge)
            pilotLastTranslationalActionTime = 0;
        else
            pilotLastTranslationalActionTime += dt;

        final double currentDesiredPositionX = ((Math.abs(pilotTranslationalCommand.getX()) < zeroJudge && Math.abs(pilotTranslationalCommand.getY()) > zeroJudge) // if the pilot moves the y axis is moving but not the x axis
                || pilotLastTranslationalActionTime > timeToStartDecelerate) // or, if there haven't been actions for a period of time
                ? currentDesiredPosition.getX() // maintain current x position
                : chassis.getChassisEncoderPosition().getX(), // otherwise, do speed control only by setting desired position to actual position (ignore proportion part)

                currentDesiredPositionY = ((Math.abs(pilotTranslationalCommand.getY()) < zeroJudge && Math.abs(pilotTranslationalCommand.getX()) > zeroJudge)
                        || pilotLastTranslationalActionTime > timeToStartDecelerate)
                        ? currentDesiredPosition.getY() : chassis.getChassisEncoderPosition().getY(); // same method

        currentDesiredPosition = new Vector2D(new double[]{currentDesiredPositionX, currentDesiredPositionY});

        if (!independentEncodersAvailable && controlMode == ControlMode.ENCODER_ASSISTED_FIELD_ORIENTATED) // if tries to use encoder when unavailable
            controlMode = ControlMode.MANUAL; // switch back to manual
        debugMessages.put("control mode", controlMode);
        switch (controlMode) {
            case MANUAL_FIELD_ORIENTATED: {
                chassis.setOrientationMode(Chassis.OrientationMode.FIELD_ORIENTATED, this);
                break;
            }
            case MANUAL: {
                chassis.setOrientationMode(Chassis.OrientationMode.ROBOT_ORIENTATED, this);
                break;
            }
            case ENCODER_ASSISTED_FIELD_ORIENTATED: {
                translationalTaskByPilotStickControl = new Chassis.ChassisTranslationalTask(
                        Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                        currentDesiredPosition.addBy(pilotTranslationalCommand.multiplyBy(targetDistanceAtMaxDesiredSpeed))
                );
            }
        }

        /* visual navigation */
        chassis.setLowSpeedModeEnabled(driverController.keyOnHold(RobotConfig.KeyBindings.processVisualApproachButton));
        final boolean processVisualApproach = driverController.keyOnHold(RobotConfig.KeyBindings.processVisualApproachButton) && visualNavigationSupported,
                initiateVisualApproach = driverController.keyOnPressed(RobotConfig.KeyBindings.processVisualApproachButton) && visualNavigationSupported;
        if (driverController.keyOnReleased(RobotConfig.KeyBindings.processVisualApproachButton))
            this.visualTaskStatus = VisualTaskStatus.FINISHED;
        if (initiateVisualApproach)
            this.initiateWallApproach();
        if (processVisualApproach)
            this.processVisualNavigationTask(dt);
        else
            this.visualTaskStatus = VisualTaskStatus.UNUSED;
        if (visualTaskStatus == VisualTaskStatus.UNUSED || visualTaskStatus == VisualTaskStatus.FINISHED)
            chassis.setTranslationalTask(translationalTaskByPilotStickControl, this);
        debugMessages.put("chassis task finished", chassis.isCurrentTranslationalTaskRoughlyComplete());
        debugMessages.put("visual task status", visualTaskStatus);

        /* <--rotation--> */
        double pilotRotationalCommand = driverController.getRotationStickValue();
        Chassis.ChassisRotationalTask rotationalTaskByPilotStick = new Chassis.ChassisRotationalTask(
                Chassis.ChassisRotationalTask.ChassisRotationalTaskType.SET_ROTATIONAL_SPEED,
                pilotRotationalCommand
        );

        // TODO make the two buttons in robot config
        if (driverController.keyOnPressed(RobotConfig.XboxControllerKey.LEFT_STICK_BUTTON))
            rotationWhenStickPressed = chassis.getYaw();
        if (driverController.keyOnHold(RobotConfig.XboxControllerKey.LEFT_STICK_BUTTON))
            rotationalTaskByPilotStick = new Chassis.ChassisRotationalTask(
                    Chassis.ChassisRotationalTask.ChassisRotationalTaskType.GO_TO_ROTATION,
                    rotationWhenStickPressed
            );
        if (driverController.keyOnHold(RobotConfig.XboxControllerKey.DPAD_DOWN))
            rotationalTaskByPilotStick = new Chassis.ChassisRotationalTask(
                    Chassis.ChassisRotationalTask.ChassisRotationalTaskType.GO_TO_ROTATION,
                    0
            );

        if (this.visualTaskStatus == VisualTaskStatus.FINISHED || this.visualTaskStatus == VisualTaskStatus.UNUSED)
            chassis.setRotationalTask(rotationalTaskByPilotStick, this);

        if (driverController.keyOnHold(RobotConfig.KeyBindings.resetIMUKey))
            chassis.resetYaw(this);

        if (driverController.keyOnPressed(RobotConfig.KeyBindings.toggleSpeedControlButton))
            chassis.setWheelSpeedControlEnabled((!chassis.isWheelSpeedControlEnabled()) && (!independentEncodersAvailable), this); // where there is independent encoder, no way we will use speed control

        if (driverController.keyOnPressed(RobotConfig.KeyBindings.toggleChassisDriveModeButton)) nextControlMode();
    }

    enum VisualTaskStatus {
        UNUSED,
        VISUAL_ROUGH_APPROACH,
        TOF_PRECISE_APPROACH,
        MAINTAIN_AND_AIM,
        FINISHED
    }
    private VisualTaskStatus visualTaskStatus;
    private BezierCurve currentVisualRoughApproachPath = null;
    private double timeNeededToArrive = 0;
    private double time = 0;
    private boolean lastAimSucceeded = true;

    private Vector2D previousWallPosition;
    private long timeTOFStageInitiated = -1;
    private boolean targetSeen = false;
    private void processVisualNavigationTask(double dt) {
        debugMessages.put("previous aim ", lastAimSucceeded ? "succeeded" : "failed");
        switch (visualTaskStatus) {
            case UNUSED: {
                initiateWallApproach();
                return;
            }
            case VISUAL_ROUGH_APPROACH: {
                if (chassis.isCurrentTranslationalTaskRoughlyComplete() && chassis.isCurrentRotationalTaskComplete()) {
                    this.timeTOFStageInitiated = System.currentTimeMillis();
                    this.targetSeen = false;
                    this.visualTaskStatus = VisualTaskStatus.TOF_PRECISE_APPROACH;
                    this.previousWallPosition = new Vector2D(new double[]{chassis.getCurrentTranslationalTask().getTranslationalValue().getX(), 0});
                    processVisualNavigationTask(0); // go immediately
                }
                return;
            }
            case TOF_PRECISE_APPROACH: {
                targetSeen |= chassis.isVisualNavigationAvailable();
                final boolean noSignOfWall = !targetSeen && System.currentTimeMillis() - timeTOFStageInitiated > RobotConfig.VisualNavigationConfigs.maxTimeToWaitForVisualNavigationMS; // if still no sign of the wall after 500ms
                if (noSignOfWall ||
                        (!processTOFPreciseGoToPosition(RobotConfig.VisualNavigationConfigs.targetedRelativePositionToWallPreciseTOFApproach, RobotConfig.VisualNavigationConfigs.distanceSensorMaxDistance))) {
                    aimFail();
                    return;
                }
                if (chassis.isCurrentTranslationalTaskComplete()) // if the difference lies with tolerance, and that the chassis reports that current task is finished
                {
                    this.visualTaskStatus = VisualTaskStatus.MAINTAIN_AND_AIM; // end of this stage
                }
                return;
            }
            case MAINTAIN_AND_AIM: {
                updateWallPositionTOF(RobotConfig.VisualNavigationConfigs.distanceSensorMaxDistance_maintainAndAim);
                double pilotXCommand = driverController.getTranslationStickVector().getX()
                        * RobotConfig.ChassisConfigs.lowSpeedModeMaximumMotorSpeedConstrain / 2;

                /* do not go beyond the x-bias limit */
                final Vector2D relativeEncoderPositionToWall = previousWallPosition.addBy(chassis.getChassisEncoderPosition().multiplyBy(-1));
                if (pilotXCommand < 0 && relativeEncoderPositionToWall.getX() > RobotConfig.VisualNavigationConfigs.maximumXBiasToWallCenterDuringAimingCM)
                    pilotXCommand = 0;
                else if (pilotXCommand > 0 && relativeEncoderPositionToWall.getX() < -RobotConfig.VisualNavigationConfigs.maximumXBiasToWallCenterDuringAimingCM)
                    pilotXCommand = 0;

                /* send pilot's x command, and the maintain distance y command by tof sensor, to the chassis */
                final Vector2D aimTargetEncoder = new Vector2D(new double[] {
                        pilotXCommand * targetDistanceAtMaxDesiredSpeed + chassis.getChassisEncoderPosition().getX(),
                        previousWallPosition.addBy(RobotConfig.VisualNavigationConfigs.targetedRelativePositionToWallPreciseTOFApproach).getY()
                });
                chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                        aimTargetEncoder), this);

                /* keep on maintaining rotation */
                chassis.setRotationalTask(new Chassis.ChassisRotationalTask(Chassis.ChassisRotationalTask.ChassisRotationalTaskType.GO_TO_ROTATION,
                        0), this);

                debugMessages.put("received dt", dt);
                debugMessages.put("maintain and aim target (enc, field)", aimTargetEncoder);
            }
        }
    }

    /**
     * @return whether navigation is trustable
     * */
    private boolean processTOFPreciseGoToPosition(Vector2D desiredWallPosition, double distanceSensorMaxDistance) {
        boolean distanceSensorFails = updateWallPositionTOF(distanceSensorMaxDistance);
        chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                previousWallPosition.addBy(desiredWallPosition)
        ), null);
        this.currentDesiredPosition = chassis.getChassisEncoderPosition(); // so the robot does not jump back to the starting point
        return !distanceSensorFails;
    }

    private boolean updateWallPositionTOF(double distanceSensorMaxDistance) {
        final double distanceSensorReading = distanceSensor.getDistance(DistanceUnit.CM),
                newWallPositionX = chassis.isVisualNavigationAvailable() ?
                        chassis.getChassisEncoderPosition().getX() - chassis.getRelativeFieldPositionToWall().getX():
                        previousWallPosition.getX();
        final Vector2D newWallPosition = new Vector2D(
                new double[]{
                        newWallPositionX,
                        chassis.getChassisEncoderPosition().getY() + distanceSensorReading
                });

        final boolean distanceSensorFails = distanceSensorReading > distanceSensorMaxDistance || distanceSensorReading < RobotConfig.VisualNavigationConfigs.distanceSensorMinDistance;
        if (!distanceSensorFails && Vector2D.displacementToTarget(previousWallPosition, newWallPosition).getMagnitude() > RobotConfig.VisualNavigationConfigs.errorTolerance / 2)
            previousWallPosition = newWallPosition; // only update if outside tolerance
        return distanceSensorFails;
    }


    private void initiateWallApproach() {
        if (!chassis.isVisualNavigationAvailable()) {
            this.visualTaskStatus = VisualTaskStatus.UNUSED;
            return;
        }

        chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                chassis.getChassisEncoderPosition().addBy(
                        RobotConfig.VisualNavigationConfigs.targetedRelativePositionToWallRoughApproach.addBy(
                                chassis.getRelativeFieldPositionToWall().multiplyBy(-1)
                ))), this);

        chassis.setRotationalTask(new Chassis.ChassisRotationalTask(
                Chassis.ChassisRotationalTask.ChassisRotationalTaskType.GO_TO_ROTATION,
                0
        ), this);
        this.visualTaskStatus = VisualTaskStatus.VISUAL_ROUGH_APPROACH;
        this.lastAimSucceeded = true;
    }

    private void aimFail() {
        this.lastAimSucceeded = false;
        this.visualTaskStatus = VisualTaskStatus.FINISHED;
        currentDesiredPosition = chassis.getChassisEncoderPosition();
    }

    /**
     * called when visual navigation is
     * the newer one is an easier and better approach
     * */
    @Deprecated
    private void processVisualNavigationTask_old(double dt) {
        switch (visualTaskStatus) {
            case UNUSED: {
                initiateWallApproach_old();
                return;
            }
            case VISUAL_ROUGH_APPROACH: {
                time += dt;
                chassis.updateDesiredTranslationInVisualNavigation(currentVisualRoughApproachPath.getPositionWithLERP(time / timeNeededToArrive), this);

                if (chassis.isCurrentTranslationalTaskComplete() && time > timeNeededToArrive)
                    this.visualTaskStatus = VisualTaskStatus.TOF_PRECISE_APPROACH;
                this.currentDesiredPosition = chassis.getChassisEncoderPosition(); // update the desired position so the robot do not move back to where it was after aim completed
                return;
            }
            case TOF_PRECISE_APPROACH: {
                // TODO write this part
                this.visualTaskStatus = VisualTaskStatus.MAINTAIN_AND_AIM;
                return;
            }
            case MAINTAIN_AND_AIM: {
                // TODO write this part
                this.visualTaskStatus = VisualTaskStatus.FINISHED;
                return;
            }
        }
    }

    /**
     * called at the the start of visual approach
     * */
    @Deprecated
    private void initiateWallApproach_old() {
        if (!chassis.isVisualNavigationAvailable()) {
            this.visualTaskStatus = VisualTaskStatus.UNUSED;
            return;
        }

        Vector2D startingFieldPositionRelativeToWall = chassis.getRelativeFieldPositionToWall();
        Vector2D differenceToTargetVector = Vector2D.displacementToTarget(startingFieldPositionRelativeToWall, RobotConfig.VisualNavigationConfigs.targetedRelativePositionToWallRoughApproach);
        this.currentVisualRoughApproachPath = new BezierCurve(
                startingFieldPositionRelativeToWall,
                startingFieldPositionRelativeToWall.addBy(
                        new Vector2D(0, differenceToTargetVector.getX())
                                .multiplyBy(RobotConfig.VisualNavigationConfigs.approachPathSmoothOutPercent)),
                RobotConfig.VisualNavigationConfigs.targetedRelativePositionToWallRoughApproach.addBy(
                        new Vector2D(Math.PI * 3/2, differenceToTargetVector.getY())
                                .multiplyBy(RobotConfig.VisualNavigationConfigs.approachPathSmoothOutPercent)
                        ),
                RobotConfig.VisualNavigationConfigs.targetedRelativePositionToWallRoughApproach
        );
        this.timeNeededToArrive = differenceToTargetVector.getMagnitude() / RobotConfig.VisualNavigationConfigs.visualApproachSpeed;
        this.time = 0;

        chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_VISUAL,
                startingFieldPositionRelativeToWall
        ), this);
        this.visualTaskStatus = VisualTaskStatus.VISUAL_ROUGH_APPROACH;

        if (RobotConfig.VisualNavigationConfigs.faceToTargetWhenApproaching)
            chassis.setRotationalTask(new Chassis.ChassisRotationalTask(Chassis.ChassisRotationalTask.ChassisRotationalTaskType.FACE_NAVIGATION_REFERENCES, 0), this);
    }

    @Override
    public void onDestroy() {

    }

    @Override
    public void reset() {
        this.chassis.gainOwnerShip(this);
        this.currentDesiredPosition = new Vector2D();
        this.rotationWhenStickPressed = 0;
        this.pilotLastTranslationalActionTime = 0;
        this.currentDesiredPosition = chassis.getChassisEncoderPosition();
        this.controlMode = RobotConfig.ControlConfigs.defaultControlMode;
        visualTaskStatus = VisualTaskStatus.UNUSED;
        if (independentEncodersAvailable) chassis.setWheelSpeedControlEnabled(false, this);
    }

    @Override
    public Map<String, Object> getDebugMessages() {
        return this.debugMessages;
    }

    private void nextControlMode() {
        switch (controlMode) {
            case MANUAL : {
                controlMode = ControlMode.MANUAL_FIELD_ORIENTATED;
                break;
            } case MANUAL_FIELD_ORIENTATED : {
                controlMode = ControlMode.ENCODER_ASSISTED_FIELD_ORIENTATED;
                break;
            } case ENCODER_ASSISTED_FIELD_ORIENTATED: {
                controlMode = ControlMode.MANUAL;
                break;
            }
        }
    }
}
