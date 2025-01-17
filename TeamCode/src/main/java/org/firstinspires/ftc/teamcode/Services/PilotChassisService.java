package org.firstinspires.ftc.teamcode.Services;

import static org.firstinspires.ftc.teamcode.RobotConfig.ChassisConfigs.timeToStartDecelerateTranslation;
import static org.firstinspires.ftc.teamcode.RobotConfig.ChassisConfigs.targetDistanceAtMaxDesiredSpeed;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.FixedAnglePixelCamera;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.HardwareUtils.ThreadedSensor;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.BezierCurve;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.PixelCameraAimBotLegacy;
import org.firstinspires.ftc.teamcode.Utils.RobotService;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.DriverGamePad;

import java.util.HashMap;
import java.util.Map;

public class PilotChassisService extends RobotService {
    private final Chassis chassis;
    private final DriverGamePad driverController;
    private final Gamepad copilotGamePad;
    public final ThreadedSensor distanceSensor;
    private double rotationMaintenanceFacing;
    private Vector2D currentDesiredPosition;
    /** time since last translational command sent by pilot */
    private double pilotLastTranslationalXActionTime, pilotLastTranslationalYActionTime, pilotLastRotationalActionTime;

    public enum ControlMode {
        MANUAL,
        MANUAL_FIELD_ORIENTATED,
        ENCODER_ASSISTED_FIELD_ORIENTATED
    }
    private ControlMode controlMode;

    private Map<String, Object> debugMessages = new HashMap<>(1);
    private int aimCenter = 0;
    private final Rotation2D pilotFacingRotation;

    private double desiredScoringHeight;
    public PilotChassisService(Chassis chassis, DriverGamePad driverController, Gamepad copilotGamePad, ThreadedSensor distanceSensor, double pilotFacing) {
        this.chassis = chassis;
        this.driverController = driverController;
        this.copilotGamePad = copilotGamePad;
        this.distanceSensor = distanceSensor;
        this.pilotFacingRotation = new Rotation2D(pilotFacing);
    }
    @Override
    public void init() {
        this.reset();
    }

    @Override
    public void reset() {
        this.chassis.gainOwnerShip(this);
        this.currentDesiredPosition = new Vector2D();
        this.rotationMaintenanceFacing = 0;
        this.pilotLastTranslationalXActionTime = this.pilotLastTranslationalYActionTime = this.pilotLastRotationalActionTime = 0;
        this.currentDesiredPosition = chassis.getChassisEncoderPosition();
        this.controlMode = RobotConfig.ControlConfigs.defaultControlMode;
        visualTaskStatus = VisualTaskStatus.UNUSED;
        aimCenter = 0;
        this.desiredScoringHeight = 1;
    }

    @Override
    public void periodic(double dt) {
        /* <--translation--> */
        final double zeroJudge = 0.001;
        Vector2D pilotTranslationalCommand = driverController.getTranslationStickVector();
        if (driverController.keyOnHold(RobotConfig.KeyBindings.processVisualApproachButton))
            pilotTranslationalCommand.multiplyBy(RobotConfig.ChassisConfigs.lowSpeedModeStickSensitivity);
        if (controlMode != ControlMode.MANUAL)
            pilotTranslationalCommand = pilotTranslationalCommand.multiplyBy(pilotFacingRotation);
        Chassis.ChassisTranslationalTask translationalTaskByPilotStickControl = new Chassis.ChassisTranslationalTask(
                Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY,
                pilotTranslationalCommand
        );

        if (Math.abs(pilotTranslationalCommand.getX()) > zeroJudge)
            pilotLastTranslationalXActionTime = 0;
        else
            pilotLastTranslationalXActionTime += dt;

        if (Math.abs(pilotTranslationalCommand.getY()) > zeroJudge)
            pilotLastTranslationalYActionTime = 0;
        else
            pilotLastTranslationalYActionTime += dt;

        final double currentDesiredPositionX = ((Math.abs(pilotTranslationalCommand.getX()) < zeroJudge && Math.abs(pilotTranslationalCommand.getY()) > zeroJudge) // if the pilot moves the y axis is moving but not the x axis
                || pilotLastTranslationalXActionTime > timeToStartDecelerateTranslation) // or, if there haven't been actions for a period of time
                ? currentDesiredPosition.getX() // maintain current x position
                : chassis.getChassisEncoderPosition().getX(), // otherwise, do speed control only by setting desired position to actual position (ignore proportion part)

                currentDesiredPositionY = ((Math.abs(pilotTranslationalCommand.getY()) < zeroJudge && Math.abs(pilotTranslationalCommand.getX()) > zeroJudge)
                        || pilotLastTranslationalYActionTime > timeToStartDecelerateTranslation)
                        ? currentDesiredPosition.getY() : chassis.getChassisEncoderPosition().getY(); // same method

        currentDesiredPosition = new Vector2D(new double[]{currentDesiredPositionX, currentDesiredPositionY});

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

        /* visual navigation: wall */
        chassis.setLowSpeedModeEnabled(driverController.keyOnHold(RobotConfig.KeyBindings.processVisualApproachButton));
        distanceSensor.setEnabled(driverController.keyOnHold(RobotConfig.KeyBindings.processVisualApproachButton));
        final boolean processVisualApproach = driverController.keyOnHold(RobotConfig.KeyBindings.processVisualApproachButton),
                initiateVisualApproach = driverController.keyOnHold(RobotConfig.KeyBindings.processVisualApproachButton) && this.visualTaskStatus == VisualTaskStatus.UNUSED;
        if (driverController.keyOnReleased(RobotConfig.KeyBindings.processVisualApproachButton))
            aimFinish();

        if (driverController.keyOnPressed(RobotConfig.KeyBindings.setAimPositionLeftButton))
            aimCenter--;
        if (driverController.keyOnPressed(RobotConfig.KeyBindings.setAimPositionRightButton))
            aimCenter++;
        final int aimCenterMax = RobotConfig.VisualNavigationConfigs.aimHorizontalPositions.length;
        if (aimCenter <= -aimCenterMax) aimCenter = -aimCenterMax + 1;
        else if (aimCenter >= aimCenterMax) aimCenter = aimCenterMax - 1;
        debugMessages.put("aim center", aimCenter);

        final double aimCenterCM = Math.copySign(RobotConfig.VisualNavigationConfigs.aimHorizontalPositions[Math.abs(aimCenter)], aimCenter);
        if (initiateVisualApproach)
            this.initiateWallApproach();
        if (processVisualApproach)
            this.processVisualNavigationTask(dt, aimCenterCM);
        else
            this.visualTaskStatus = VisualTaskStatus.UNUSED;
        debugMessages.put("visual task status", visualTaskStatus);

        /* send pilot command to chassis if both types of visual navigation are unused */
        if (getSlowMotionButtonsMovement().getMagnitude() > 0.05) {
            chassis.setOrientationMode(Chassis.OrientationMode.ROBOT_ORIENTATED, this);
            chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                            Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.SET_VELOCITY,
                            getSlowMotionButtonsMovement()),
                    this);
            pilotLastTranslationalXActionTime = 0;
        } else
            if ((visualTaskStatus == VisualTaskStatus.UNUSED || visualTaskStatus == VisualTaskStatus.FINISHED))
                chassis.setTranslationalTask(translationalTaskByPilotStickControl, this);

        /* <--rotation--> */
        double pilotRotationalCommand = driverController.getRotationStickValue();
        if (Math.abs(pilotRotationalCommand) > 0.05)
            pilotLastRotationalActionTime = 0;
        else pilotLastRotationalActionTime += dt;

        Chassis.ChassisRotationalTask rotationalTaskByPilotStick = new Chassis.ChassisRotationalTask(
                Chassis.ChassisRotationalTask.ChassisRotationalTaskType.SET_ROTATIONAL_SPEED,
                pilotRotationalCommand
        );

        /* auto facing control*/
        if (driverController.keyOnHold(RobotConfig.KeyBindings.facePilotLeftButton))
            rotationMaintenanceFacing = Math.toRadians(90) + pilotFacingRotation.getRadian();
        else if (driverController.keyOnHold(RobotConfig.KeyBindings.facePilotRightButton))
            rotationMaintenanceFacing = Math.toRadians(270) + pilotFacingRotation.getRadian();
        else if (driverController.keyOnHold(RobotConfig.KeyBindings.facePilotFrontButton))
            rotationMaintenanceFacing = Math.toRadians(0) + pilotFacingRotation.getRadian();
        else if (driverController.keyOnHold(RobotConfig.KeyBindings.facePilotBackButton))
            rotationMaintenanceFacing = Math.toRadians(180) + pilotFacingRotation.getRadian();

        if (pilotLastRotationalActionTime < RobotConfig.ChassisConfigs.timeToStartDecelerateRotation)
            rotationMaintenanceFacing = chassis.getYaw();
        else if (!driverController.keyOnHold(RobotConfig.KeyBindings.turnOffRotationControlButton))
            rotationalTaskByPilotStick = new Chassis.ChassisRotationalTask(
                    Chassis.ChassisRotationalTask.ChassisRotationalTaskType.GO_TO_ROTATION,
                    rotationMaintenanceFacing);

        /* if there is no visual task going, send the pilot's rotation command to chassis module */
        if ((visualTaskStatus == VisualTaskStatus.UNUSED || visualTaskStatus == VisualTaskStatus.FINISHED))
            chassis.setRotationalTask(rotationalTaskByPilotStick, this);

        if (driverController.keyOnHold(RobotConfig.KeyBindings.resetIMUKey))
            chassis.resetYaw(this);

        if (driverController.keyOnPressed(RobotConfig.KeyBindings.toggleChassisDriveModeButton)) nextControlMode();
    }

    /**
     *
     * @return the movement. in robot's orientation
     * */
    private Vector2D getSlowMotionButtonsMovement() {
        Vector2D movementBySlowMotionButtons = new Vector2D();

        movementBySlowMotionButtons = movementBySlowMotionButtons.addBy(driverController.keyOnHold(RobotConfig.KeyBindings.moveLeftSlowlyButton) ? new Vector2D(Math.PI, RobotConfig.KeyBindings.slowMovementButtonSensitivity): new Vector2D());
        movementBySlowMotionButtons = movementBySlowMotionButtons.addBy(driverController.keyOnHold(RobotConfig.KeyBindings.moveRightSlowlyButton) ? new Vector2D(0, RobotConfig.KeyBindings.slowMovementButtonSensitivity): new Vector2D());
        movementBySlowMotionButtons = movementBySlowMotionButtons.addBy(driverController.keyOnHold(RobotConfig.KeyBindings.moveBackSlowlyButton) ? new Vector2D(-Math.PI/2, RobotConfig.KeyBindings.slowMovementButtonSensitivity): new Vector2D());
        movementBySlowMotionButtons = movementBySlowMotionButtons.addBy(driverController.keyOnHold(RobotConfig.KeyBindings.moveForwardSlowlyButton) ? new Vector2D(Math.PI/2, RobotConfig.KeyBindings.slowMovementButtonSensitivity): new Vector2D());

        return movementBySlowMotionButtons;
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
    private Vector2D wallFieldPositionForRoughApproach = null;
    private void processVisualNavigationTask(double dt, double aimCenterCM) {
        switch (visualTaskStatus) {
            case UNUSED: {
                initiateWallApproach();
                return;
            }
            case VISUAL_ROUGH_APPROACH: {
                final Vector2D targetedRelativePositionToWall = RobotConfig.VisualNavigationConfigs.targetedRelativePositionToWallRoughApproach.addBy(
                        new Vector2D(new double[] {aimCenterCM, 0})
                );
                chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(
                        Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                        wallFieldPositionForRoughApproach.addBy(targetedRelativePositionToWall)), this);

                chassis.setRotationalTask(new Chassis.ChassisRotationalTask(
                        Chassis.ChassisRotationalTask.ChassisRotationalTaskType.GO_TO_ROTATION,
                        0
                ), this);

                if (chassis.isCurrentTranslationalTaskRoughlyComplete() && chassis.isCurrentRotationalTaskRoughlyComplete()) {
                    this.timeTOFStageInitiated = System.currentTimeMillis();
                    this.targetSeen = false;
                    this.visualTaskStatus = VisualTaskStatus.TOF_PRECISE_APPROACH;
                    this.previousWallPosition = new Vector2D(new double[]{chassis.getCurrentTranslationalTask().getTranslationalValue().getX(), 0});
                    processVisualNavigationTask(0, aimCenter); // go immediately
                }
                return;
            }
            case TOF_PRECISE_APPROACH: {
                if (!goToWallPrecise(aimCenterCM))
                    aimFail();
                final boolean stickToWallAdjustmentDemanded = Math.abs(driverController.getTranslationStickVector().getX()) > 0.1
                        || driverController.keyOnHold(RobotConfig.KeyBindings.moveAimingPositionLeftManuallyButton)
                        || driverController.keyOnHold(RobotConfig.KeyBindings.moveAimingPositionRightManuallyButton);
                if (chassis.isCurrentTranslationalTaskComplete()
                        // && stickToWallAdjustmentDemanded
                ) // if the difference lies with tolerance, and that the chassis reports that current task is finished
                    this.visualTaskStatus = VisualTaskStatus.MAINTAIN_AND_AIM; // end of this stage
                return;
            }
            case MAINTAIN_AND_AIM:
                stickToWallAndManualAdjust();
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
        final Vector2D relativeFieldPositionToWall = chassis.getRelativeFieldPositionToWall();
        final double distanceSensorReading = distanceSensor.getSensorReading(),
                newWallPositionX = relativeFieldPositionToWall != null?
                        chassis.getChassisEncoderPosition().getX() - relativeFieldPositionToWall.getX():
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
        final Vector2D relativePositionToWall;
        if ((relativePositionToWall = chassis.getRelativeFieldPositionToWall()) == null) {
            this.visualTaskStatus = VisualTaskStatus.UNUSED;
            return;
        }
        final Vector2D wallRelativePositionToRobot = relativePositionToWall.multiplyBy(-1);
        wallFieldPositionForRoughApproach = chassis.getChassisEncoderPosition().addBy(wallRelativePositionToRobot);
        this.visualTaskStatus = VisualTaskStatus.VISUAL_ROUGH_APPROACH;
        this.lastAimSucceeded = true;
        this.scoringChassisFeedForwardAmountCM = 0;
    }

    private boolean goToWallPrecise(double horizontalAimingTargetFromCenter) {
        final Vector2D targetedRelativePositionToWall = getDesiredPreciseWallAimPosition()
                .addBy(new Vector2D(new double[] {horizontalAimingTargetFromCenter, 0}));

        targetSeen |= chassis.isVisualNavigationAvailable();
        final boolean noSignOfWall = !targetSeen && System.currentTimeMillis() - timeTOFStageInitiated > RobotConfig.VisualNavigationConfigs.maxTimeToWaitForVisualNavigationMS; // if still no sign of the wall after 500ms
        return !noSignOfWall &&
                (processTOFPreciseGoToPosition(
                        targetedRelativePositionToWall,
                        RobotConfig.VisualNavigationConfigs.distanceSensorMaxDistance));
    }

    private void stickToWallAndManualAdjust() {
        updateWallPositionTOF(RobotConfig.VisualNavigationConfigs.distanceSensorMaxDistance_maintainAndAim);

        double pilotXCommand = getPilotXCommandDuringAdjustment();
        /* do not go beyond the x-bias limit */
        final Vector2D relativeEncoderPositionToWall = previousWallPosition.addBy(chassis.getChassisEncoderPosition().multiplyBy(-1));
        if (pilotXCommand < 0 && relativeEncoderPositionToWall.getX() > RobotConfig.VisualNavigationConfigs.maximumXBiasToWallCenterDuringAimingCM)
            pilotXCommand = 0;
        else if (pilotXCommand > 0 && relativeEncoderPositionToWall.getX() < -RobotConfig.VisualNavigationConfigs.maximumXBiasToWallCenterDuringAimingCM)
            pilotXCommand = 0;

        /* send pilot's x command, and the maintain distance y command by tof sensor, to the chassis */
        final Vector2D aimTargetEncoder = new Vector2D(new double[] {
                pilotXCommand * targetDistanceAtMaxDesiredSpeed + chassis.getChassisEncoderPosition().getX(),
                previousWallPosition.getY() - getDesiredScoringDistanceToWall()
        });
        chassis.setTranslationalTask(new Chassis.ChassisTranslationalTask(Chassis.ChassisTranslationalTask.ChassisTranslationalTaskType.DRIVE_TO_POSITION_ENCODER,
                aimTargetEncoder), this);

        /* keep on maintaining rotation */
        chassis.setRotationalTask(new Chassis.ChassisRotationalTask(Chassis.ChassisRotationalTask.ChassisRotationalTaskType.GO_TO_ROTATION,
                0), this);
    }

    private double getPilotXCommandDuringAdjustment() {
        final double copilotXCommand = copilotGamePad.right_stick_x;
        if (Math.abs(copilotXCommand) > 0.05)
            return copilotXCommand;

        double pilotXCommand = driverController.getTranslationStickVector().getX() * RobotConfig.ControlConfigs.pilotController_translationStickXPreciseAimSensitivity;

        if (driverController.keyOnHold(RobotConfig.KeyBindings.moveAimingPositionLeftManuallyButton))
            pilotXCommand -= RobotConfig.ControlConfigs.pilotController_translationStickXPreciseAimSensitivity;
        if (driverController.keyOnHold(RobotConfig.KeyBindings.moveAimingPositionRightManuallyButton))
            pilotXCommand += RobotConfig.ControlConfigs.pilotController_translationStickXPreciseAimSensitivity;

        return pilotXCommand;
    }

    private Vector2D getDesiredPreciseWallAimPosition() {
        return new Vector2D(new double[] {
                RobotConfig.VisualNavigationConfigs.targetedRelativePositionToWallPreciseTOFApproach.getX(),
                -getDesiredScoringDistanceToWall()
        }).addBy(new Vector2D(new double[] {
                0,
                scoringChassisFeedForwardAmountCM
        }));
    }

    private double getDesiredScoringDistanceToWall() {
        return RobotConfig.ArmConfigs.distancesToWallAccordingToScoringHeight.getYPrediction(desiredScoringHeight);
    }

    private void aimFail() {
        this.lastAimSucceeded = false;
        aimFinish();
    }

    private void aimFinish() {
        if (previousWallPosition != null)
            this.currentDesiredPosition = previousWallPosition.addBy(RobotConfig.VisualNavigationConfigs.targetedRelativePositionToWallRoughApproach);
        this.rotationMaintenanceFacing = 0;
        this.visualTaskStatus = VisualTaskStatus.FINISHED;
    }

    /** called by upper structure service */
    public void setDesiredScoringHeight(double desiredScoringHeight) {
        this.desiredScoringHeight = desiredScoringHeight;
    }

    double scoringChassisFeedForwardAmountCM = 0;
    public void setScoringHeightMovingDirection(double scoringHeightMovingDirection) {
        if (scoringHeightMovingDirection > 0)
            scoringChassisFeedForwardAmountCM = 15 * scoringHeightMovingDirection;
        else if (scoringHeightMovingDirection < 0)
            scoringChassisFeedForwardAmountCM = 20 * scoringHeightMovingDirection;
        else
            scoringChassisFeedForwardAmountCM = 0;
    }

    public double getActualScoringHeightAccordingToDistanceToWall(double defaultScoringHeight) {
        if (this.visualTaskStatus != VisualTaskStatus.MAINTAIN_AND_AIM) return defaultScoringHeight;
        debugMessages.put("scoring height by distance sensor ", RobotConfig.ArmConfigs.scoringHeightAccordingToActualDistanceToWall.getYPrediction(distanceSensor.getSensorReading()));
        return RobotConfig.ArmConfigs.scoringHeightAccordingToActualDistanceToWall.getYPrediction(distanceSensor.getSensorReading());
    }


    public boolean stickToWallComplete() {
        return this.visualTaskStatus == VisualTaskStatus.MAINTAIN_AND_AIM;
    }

    @Override
    public void onDestroy() {

    }

    @Override
    public Map<String, Object> getDebugMessages() {
        debugMessages.put("distance sensor reading", distanceSensor.getSensorReading());
        debugMessages.put("desired scoring distance", getDesiredScoringDistanceToWall());
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
