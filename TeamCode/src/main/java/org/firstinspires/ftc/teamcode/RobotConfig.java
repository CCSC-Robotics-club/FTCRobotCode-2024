package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Modules.TripleIndependentEncoderAndIMUPositionEstimator;
import org.firstinspires.ftc.teamcode.Services.PilotChassisService;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.FixedAngleCameraProfile;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.LookUpTable;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.ArmGravityController;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.EnhancedPIDController;
import org.firstinspires.ftc.teamcode.Utils.ComputerVisionUtils.TeamElementFinderTensorflow;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;

import java.util.HashMap;
import java.util.Map;

public final class RobotConfig {
    public static final boolean useFtcDashboardForTelemetry = true;

    public static final HardwareConfigs hardwareConfigs_2024OffSeason = new HardwareConfigs( // backup machine
            new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            ),
            null,
            new TripleIndependentEncoderAndIMUPositionEstimator.TripleIndependentEncoderAndIMUSystemParams(
                    false,
                    true,
                    false,
                    135.812,
                    -0.1023
            ),
            new String[] {"frontRight", "backLeft", "backRight"},
            true,
            false,
            true,
            false,
            false,
            false,
            false,
            false
    );
    public static final HardwareConfigs hardwareConfigs_2024Competition_backup = new HardwareConfigs( // backup machine
            new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            ),
            null,
            new TripleIndependentEncoderAndIMUPositionEstimator.TripleIndependentEncoderAndIMUSystemParams(
                    true,
                    true,
                    false, // not known yet
                    135.812,
                    -0.385
            ),
            new String[] {"backRight", "frontRight", "frontLeft"},
            true,
            false,
            true,
            false,
            false,
            false,
            false,
            false
    );
    public static final HardwareConfigs hardwareConfigs_2024Competition = new HardwareConfigs( // new robot for competition
            new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            ),
            null,
            new TripleIndependentEncoderAndIMUPositionEstimator.TripleIndependentEncoderAndIMUSystemParams(
                    true,
                    false,
                    true,
                    135.812,
                    0.378
            ),
            new String[] {"frontRight", "backRight", "frontLeft"},
            true,
            false,
            true,
            false,
            false,
            false,
            false,
            false
    );

    public static final HardwareConfigs hardwareConfigs_newer_chassis = new HardwareConfigs( // config for newer chassis
            new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                    )),
            null,
            new TripleIndependentEncoderAndIMUPositionEstimator.TripleIndependentEncoderAndIMUSystemParams(
                    false,
                    false,
                    true,
                    135.812,
                    -0.038
            ),
            new String[] {"backRight", "frontRight", "backLeft"},
            true,
            false,
            true,
            false,
            false,
            true,
            false,
            true
    );

    public static final HardwareConfigs competitionConfig = hardwareConfigs_2024OffSeason;

    public static final HardwareConfigs testConfig = hardwareConfigs_2024Competition;

    public static final class IntakeConfigsLegacy {
        public static final String intakeMotor1Name = "intake1", intakeMotor2Name = "intake2";
        public static final double intakeMotor1Power = -0.75, intakeMotor2Power = 0.75; // competition machine
//        public static final double intakeMotor1Power = -0.85, intakeMotor2Power = -0.85; // backup machine

        public static final double spewPixelDriveBackDistance = 14;
        public static final long spewPixelTimeMillis = 1000;
    }

    public static final class ArmConfigs {
        /** positive should be scoring */
        public static final boolean motor1Reversed = true, motor2Reversed = true, encoderReversed = true;

        public static final ArmGravityController.ArmProfile armProfile = new ArmGravityController.ArmProfile(
                0.8,
                90,
                0.01,
                2,
                0.15,
                120,
                0,
                20,
                600,
                450,
                0.15,
                new LookUpTable(
                        new double[] {0, 40, 70, 180, 250, 300, 350, 400},
                        new double[] {0.1, 0.15, 0.25, 0.15, 0, -0.08, -0.15, -0.25}
                )
        );

        /** simple arm controller */
        public static final double
                maxPowerWhenMovingUpNormal = 1,
                maxPowerWhenMovingDownNormal = 0.6,
                errorStartDecelerateNormal = 400,
                powerNeededToMoveUpNormal = 0.24,
                powerNeededToMoveDownNormal = 0.2,
                errorToleranceNormal = 10,
                errorAsArmInPositionNormal = 80,

                maxPowerWhenMovingUpScoring = 0.4,
                maxPowerWhenMovingDownScoring = 0.7,
                errorStartDecelerateScoring = 150,
                powerNeededToMoveUpScoring = 0.05,
                powerNeededToMoveDownScoring = 0.15,
                basePowerWhenMovingScoringHeightUp = -0.3,
                basePowerWhenMovingScoringHeightDown = 0,
                errorToleranceScoring = 12,
                        errorAsArmInPositionScoring = 60;
        public static final LookUpTable baseGravityPower = new LookUpTable(
                new double[] {1400, 1600, 2100},
                new double[] {-0.05, -0.25, -0.35}
        );

        public enum Position {
            INTAKE,
            PREPARE_TO_SCORE,
            SCORE,
        }

        public static final Map<Position, Double> encoderPositions = new HashMap<>(); // in reference to zero position (limit switch)
        static {
//            encoderPositions.put(Position.INTAKE, 0.0);
//            encoderPositions.put(Position.GRAB_STACK, 60.0);
//            encoderPositions.put(Position.GRAB_STACK_LOW, 50.0);
//            encoderPositions.put(Position.SCORE, 1200.0);
            encoderPositions.put(Position.INTAKE, 0.0);
            encoderPositions.put(Position.PREPARE_TO_SCORE, 1300.0);
        }

        private static final double[]
                scoringHeight = new double[] {0, 0.33, 0.66, 1},
                correspondingArmEncoderValues = new double[] {2100, 1780, 1680, 1640},
                correspondingExtendValues = new double[] {100, 500, 800, 1200},
                correspondingServoPositions = new double[] {0.68, 0.72, 0.8, 0.86},
                correspondingDistanceToWall = new double[] {24, 14, 8, 2};

        public static final double manualStageMinScoringHeight = 0, manualStageMaxScoringHeight = 1, inAdvanceHeight = 0.1;
        public static final LookUpTable
                armScoringAnglesAccordingToScoringHeight = new LookUpTable(scoringHeight, correspondingArmEncoderValues),
                distancesToWallAccordingToScoringHeight = new LookUpTable(scoringHeight, correspondingDistanceToWall),
                scoringHeightAccordingToActualDistanceToWall = new LookUpTable(correspondingDistanceToWall, scoringHeight),
                extendValuesAccordingToScoringHeight = new LookUpTable(scoringHeight, correspondingExtendValues),
                flipperPositionsAccordingToScoringHeight = new LookUpTable(scoringHeight, correspondingServoPositions);

        public static final double autoStageScoringDistanceToWall = 26, autoStageArmScoringHeight = 0, autoStageScoringExtendPosition = 300, autoStageScoringServoPosition = 0.62;
    }

    public static final class ExtendConfigs {
        public static final boolean extendMotorReversed = false, extendEncoderReversed = false;

        public static final double
                /* pid */
                maxPowerWhenMovingForward = 0.6,
                maxPowerWhenMovingBackward = 0.65,
                errorStartDecelerate = 200,
                powerNeededToMoveForward = 0.04,
                powerNeededToMoveBackward = 0.04,
                errorTolerance = 24,
                errorAsTaskFinished = 60,
                /* settings */
                maxExtendValue = 1450,
                intakeValue = 500,
                grabStackValue = 0;
    }

    public static final class FlippableDualClawConfigs {

        public static final double leftClawClosePosition = 0.48, leftClawOpenPosition = 0.81, rightClawClosedPosition = 0.7, rightClawOpenPosition = 0.37;

        public static final int colorDetectorThreshold = 500;
    }
    public static final class LauncherConfigs {
        public static final double launcherZeroPosition = 1,
                launcherActivatePosition = 0.3,
                liftZeroPosition = 1,
                liftActivatePosition = 0.6,
                servoMovementTime = 0.7;
    }

    public static final class ChassisConfigs {
        public static final LookUpTable wheelPowerLookUpTable = new LookUpTable(
                new double[] {0, 0.03, 0.5, 0.9, 1}, // encoder velocity / max velocity
                new double[] {0, 0.1, 0.4, 0.8, 1} // motor power
        );


        public static final double chassisRotation_maximumCorrectionPower = 0.65;
        public static final double chassisRotation_minimumCorrectionPower = 0;
        public static final double chassisRotation_errorStartDecelerateRadian = Math.toRadians(45);
        public static final double chassisRotation_errorTolerance = Math.toRadians(0);
        public static final double chassisRotationErrorAsFinished = Math.toRadians(5);
        public static final double chassisRotationErrorAsRoughlyFinished = Math.toRadians(12);
        public static final double chassisRotationSpeedAsStopped = Math.toRadians(15); // rad/sec
        public static final double chassisRotation_feedForwardDelay = 0.1; // in seconds
        public static final EnhancedPIDController.StaticPIDProfile chassisRotationControllerProfile = new EnhancedPIDController.StaticPIDProfile(
                Math.PI * 2,
                chassisRotation_maximumCorrectionPower,
                chassisRotation_minimumCorrectionPower,
                chassisRotation_errorStartDecelerateRadian,
                chassisRotation_errorTolerance,
                chassisRotation_feedForwardDelay,
                0,0
        );

        /* position estimator */
        public static final int positionEstimator_speedEstimationFrequency = 100;

        /* encoder drive-to-position PID */
        public static final EnhancedPIDController.StaticPIDProfile encoderTranslationalControllerProfileX = new EnhancedPIDController.StaticPIDProfile(
                Double.POSITIVE_INFINITY,
                1.05,
                0.03,
                26,
                1,
                0.22,
                0, 0
        );
        public static final EnhancedPIDController.StaticPIDProfile encoderTranslationalControllerProfileY = new EnhancedPIDController.StaticPIDProfile(
                Double.POSITIVE_INFINITY,
                1.05,
                0.015,
                32,
                1,
                0.26,
                0, 0
        );

        /** the maximum chassis speed when using visual navigation */
        public static final double lowSpeedModeMaximumMotorSpeedConstrain = 0.6;
        public static final double lowSpeedModeStickSensitivity = 0.45;

        /** a power constrain to the motors to not push them too hard */
        public static final double ordinaryModeMaximumMotorSpeedConstrain = 1.4;

        public static final double xPowerRate = 1.05,
                yPowerRate = 1;
        /** the distance to set the position target, when pilot sends a command of full speed  */
        public static final double targetDistanceAtMaxDesiredSpeed = 75;
        /** the smooth-out time, or the time after the pilot let the chassis stop till it start maintaining its current position */
        public static final double timeToStartDecelerateTranslation = 0.7;

        public static final double timeToStartDecelerateRotation = 0.3;

        /** within how much distance should the translational task be counted as complete */
        public static final double errorAsTaskFinishedCM = 3.5;
        public static final double chassisSpeedAsRobotStoppedCMPerSec = 20;

        public static final double errorAsTaskRoughlyFinished = 12;

        public static final double autoStageMaxVelocity = 160;
        public static final double autoStageMaxAcceleration = 200;
        public static final double autoStageMaxAngularVelocity = Math.toRadians(180);
        public static final double autoStageInAdvanceTime = 0.15;
    }

    public static final class VisualNavigationConfigs {
//        public static final FixedAngleCameraProfile visualCameraProfile = new FixedAngleCameraProfile( // newer chassis (prototype)
//                14,
//                1.16298,
//                -0.003366,
//                -0.0022232,
//                new double[] {0, 0},
//                new double[] {0,0});

        // 2024 competition chassis
        public static final FixedAngleCameraProfile visualCameraProfile = new FixedAngleCameraProfile(
                22,
                0.70,
                -0.00315,
                -0.00483,
                new double[2], new double[2]);
        public static final double visualModuleUpdateRate = 5;
        public static final long timeKeepTrackingAfterTagLostMillis = (long) 10e9;
        /**
         * the maximum amount of velocity, in cm/s, that the visual module's result is reliable
         * which is to say, the bias of the result caused by camera delay is still in an acceptable range
         *  */
        public static final double visualModuleResultReliableVelocityMax = 30;
        public static final double visualModuleResultReliableAngularSpeedMax = Math.PI / 4;

        /**
         * the error tolerance of the visual camera
         * if the updated wall encoder position differed to the current smaller than this value, do not update
         * in cm
         * */
        public static final double errorTolerance = 3;

        public static final double distanceSensorMaxDistance = 45;
        public static final double distanceSensorMaxDistance_maintainAndAim = 15;
        public static final double distanceSensorMinDistance = 0;
        public static final double approachReverseSpeedTolerance = 20; // if the robot goes away from the wall in more than 20cm/s, we think the distance sensor failed

        public static final Vector2D targetedRelativePositionToWallRoughApproach = new Vector2D(new double[]{0, -30});
        public static final Vector2D targetedRelativePositionToWallPreciseTOFApproach = new Vector2D(new double[] {0, -5});
        public static final double[] aimHorizontalPositions = new double[] {0, 4, 12, 20};
        public static final double autoStageScoringPositionsLeft = -15, autoStageScoringPositionsCenter = 5.5, autoStageScoringPositionsRight = 17.5;
        public static final double maximumXBiasToWallCenterDuringAimingCM = 20;

        public static final long maxTimeToWaitForVisualNavigationMS = 800; // after this much milliseconds, if the target still does not occur, the navigation will be considered failed



        public static final FixedAngleCameraProfile pixelCameraSetUpProfile = new FixedAngleCameraProfile(
                20,
                0.784,
                -0.001092,
                -9.19E-4,
                new double[2], new double[2]
        );
        public static final float tensorCameraMinConfident = 0.82f;
        public static final double pixelCameraInstallFacing = Math.PI;  // facing back
        public static final double pixelDetectionMaximumDistance = 50; // in cm
        public static final double pixelSearchVelocity = 100; // in cm/s
        public static final Vector2D pixelFeedingSweetSpot = new Vector2D(new double[]{0, 15});
        public static final double feedingSpotErrorTolerance = 4;
        public static final double feedingDistanceForward = -35; // in cm
        public static final long feedTimeMillis = 3000;
    }

    public enum TeamElementPosition {
        LEFT,
        CENTER,
        RIGHT,
        UNDETERMINED
    }
    public static final class TeamElementFinderConfigs {

        public static final Map<TeamElementPosition, Double[]> teamElementPositionSearchRotationRanges = new HashMap<>();
        public static final double distanceThreshold = 20;
        static {
            teamElementPositionSearchRotationRanges.put(TeamElementPosition.LEFT, new Double[]{Math.toRadians(125), Math.toRadians(55)});
            teamElementPositionSearchRotationRanges.put(TeamElementPosition.CENTER, new Double[]{Math.toRadians(35), Math.toRadians(-35)});
            teamElementPositionSearchRotationRanges.put(TeamElementPosition.RIGHT, new Double[]{Math.toRadians(-55), Math.toRadians(-125)});
        }
        public static final int searchRangePixels = 150;
        public static final int minimumSize = 0;
        public static final Vector2D expectedTargetPosition = new Vector2D();
        public static final double searchRotation = Math.toRadians(50); // in reference to the center team element position
        public static final double searchRange = Math.toRadians(15);
        public static final long timeOut = 1200; // 1200
    }

    public static final class ControlConfigs {
        /** the dead band for the controller axis, when the other axis is idle(0.0) */
        public static final double pilotController_deadBand = 0.05;
        /** the dead band for the controller axis, when the other axis is full(1.0) */
        public static final double pilotController_fullStickDeadBand = 0.25;
        public static final double pilotController_translationStickXSensitivity = 1;
        public static final double pilotController_translationStickYSensitivity = -1;
        public static final double pilotController_translationStickXPreciseAimSensitivity = 0.2; // during wall aim
        public static final double pilotController_rotationSensitivity = -0.75;

        /** the exponent of the the x axis of translational stick, 1 for linear */
        public static final double pilotController_translationStickXExp = 1.6;
        /** the exponent of the the y axis of translational stick, 1 for linear */
        public static final double pilotController_translationStickYExp = 1.6;
        /** the exponent of the rotational axis , 1 for linear */
        public static final double pilotController_rotationalStickExp = 2;


        public static final double visualNavigationAimingSensitivityCMPS = 30;
        public static final double triggerThreshold = 0.4;
        public static PilotChassisService.ControlMode defaultControlMode = PilotChassisService.ControlMode.MANUAL_FIELD_ORIENTATED;
        public static final int pilotControllerKeyUpdatingRate = 60; // times per second

        public enum ControllerStick {
            RIGHT_HAND,
            LEFT_HAND
        }
        public static final ControlConfigs.ControllerStick translationalControllerStick = ControllerStick.RIGHT_HAND;
    }

    public static final class KeyBindings {
        public static final XboxControllerKey resetIMUKey = XboxControllerKey.Y;
        public static final XboxControllerKey turnOffRotationControlButton = XboxControllerKey.YIELD;
        public static final XboxControllerKey facePilotLeftButton = XboxControllerKey.DPAD_LEFT;
        public static final XboxControllerKey facePilotRightButton = XboxControllerKey.DPAD_RIGHT;
        public static final XboxControllerKey facePilotFrontButton = XboxControllerKey.DPAD_UP;
        public static final XboxControllerKey facePilotBackButton = XboxControllerKey.DPAD_DOWN;

        public static final XboxControllerKey moveLeftSlowlyButton = XboxControllerKey.YIELD;
        public static final XboxControllerKey moveRightSlowlyButton = XboxControllerKey.YIELD;
        public static final XboxControllerKey moveForwardSlowlyButton = XboxControllerKey.YIELD;
        public static final XboxControllerKey moveBackSlowlyButton = XboxControllerKey.YIELD;
        public static final double slowMovementButtonSensitivity = 1;

        public static final XboxControllerKey toggleChassisDriveModeButton = XboxControllerKey.A;
        public static final XboxControllerKey processVisualApproachButton = XboxControllerKey.LEFT_TRIGGER;
        public static final XboxControllerKey setAimPositionLeftButton = XboxControllerKey.YIELD;
        public static final XboxControllerKey setAimPositionRightButton = XboxControllerKey.YIELD;
        public static final XboxControllerKey moveAimingPositionLeftManuallyButton = XboxControllerKey.X;
        public static final XboxControllerKey moveAimingPositionRightManuallyButton = XboxControllerKey.B;

        public static final XboxControllerKey processFaceToPixelAndFeedButton = XboxControllerKey.RIGHT_TRIGGER;
        public static final XboxControllerKey processLineUpWithPixelAndFeedButton = XboxControllerKey.RIGHT_BUMPER;
    }

    public enum XboxControllerKey {
        A, B, X, Y,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        LEFT_BUMPER, RIGHT_BUMPER,
        LEFT_TRIGGER, RIGHT_TRIGGER,
        LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON,
        YIELD
    }

    public static class HardwareConfigs { // config for newer chassis
        public final IMU.Parameters imuParameter;
        public final IMU.Parameters alternativeIMUParameter;
        public final TripleIndependentEncoderAndIMUPositionEstimator.TripleIndependentEncoderAndIMUSystemParams encodersParams;
        public final boolean frontLeftWheel_motorReversed;
        public final boolean frontRightWheel_motorReversed;
        public final boolean backLeftWheel_motorReversed;
        public final boolean backRightWheel_motorReversed;

        public final boolean frontLeftWheel_encoderReversed;
        public final boolean frontRightWheel_encoderReversed;
        public final boolean backLeftWheel_encoderReversed;
        public final boolean backRightWheel_encoderReversed;

        public final String[] encoderNames;

        public HardwareConfigs(
                IMU.Parameters imuParameter,
                IMU.Parameters alternativeIMUParameter,
                TripleIndependentEncoderAndIMUPositionEstimator.TripleIndependentEncoderAndIMUSystemParams encodersParams,
                String[] encoderNames,
                boolean frontLeftWheel_motorReversed,
                boolean frontRightWheel_motorReversed,
                boolean backLeftWheel_motorReversed,
                boolean backRightWheel_motorReversed,
                boolean frontLeftWheel_encoderReversed,
                boolean frontRightWheel_encoderReversed,
                boolean backLeftWheel_encoderReversed,
                boolean backRightWheel_encoderReversed) {

            this.imuParameter = imuParameter;
            this.alternativeIMUParameter = alternativeIMUParameter;
            this.encodersParams = encodersParams;

            this.frontLeftWheel_motorReversed = frontLeftWheel_motorReversed;
            this.frontRightWheel_motorReversed = frontRightWheel_motorReversed;
            this.backLeftWheel_motorReversed = backLeftWheel_motorReversed;
            this.backRightWheel_motorReversed = backRightWheel_motorReversed;

            this.frontLeftWheel_encoderReversed = frontLeftWheel_encoderReversed;
            this.frontRightWheel_encoderReversed = frontRightWheel_encoderReversed;
            this.backLeftWheel_encoderReversed = backLeftWheel_encoderReversed;
            this.backRightWheel_encoderReversed = backRightWheel_encoderReversed;

            this.encoderNames = encoderNames;
        }
    }
}
