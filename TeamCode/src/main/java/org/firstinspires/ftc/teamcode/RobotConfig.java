package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Modules.ArmLegacy;
import org.firstinspires.ftc.teamcode.Modules.TripleIndependentEncoderAndIMUPositionEstimator;
import org.firstinspires.ftc.teamcode.Services.PilotChassisService;
import org.firstinspires.ftc.teamcode.Utils.Claw;
import org.firstinspires.ftc.teamcode.Utils.FixedAngleCameraProfile;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.LookUpTable;
import org.firstinspires.ftc.teamcode.Utils.MechanismControllers.EnhancedPIDController;
import org.firstinspires.ftc.teamcode.Utils.TeamElementFinder;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;

import java.util.HashMap;
import java.util.Map;

public final class RobotConfig {
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
            new String[] {"frontRight", "backLeft", "frontLeft"},
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
        public static final boolean motorReversed = true, encoderReversed = true;
        public static final double
                maxPowerWhenMovingUp = 1,
                maxPowerWhenMovingDown = 0.8,
                errorStartDecelerate = 1000,
                powerNeededToMoveUp = 0.5,
                powerNeededToMoveDown = 0.4,
                errorTolerance = 100;

        private static final double[]
                scoringHeight = new double[] {0, 0.25, 0.5, 0.75, 1},
                correspondingArmEncoderValues = new double[] {5300 ,5100, 4700, 4300, 4100},
                correspondingServoPositions = new double[] {0.7, 0.76, 0.85, 0.92, 0.95},
                correspondingDistanceToWall = new double[] {30, 24, 15, 7, 2}; // in cm
        public static final LookUpTable
                armScoringAnglesAccordingToScoringHeight = new LookUpTable(scoringHeight, correspondingArmEncoderValues),
                flipperPositionsAccordingToScoringHeight = new LookUpTable(scoringHeight, correspondingServoPositions),
                distancesToWallAccordingToScoringHeight = new LookUpTable(scoringHeight, correspondingDistanceToWall);

        public enum Position {
            INTAKE,
            GRAB_STACK,
            SCORE
        }

        public static final Map<Position, Double> encoderPositions = new HashMap<>(); // in reference to zero position (limit switch)
        static {
            encoderPositions.put(Position.INTAKE, -100.0);
            encoderPositions.put(Position.GRAB_STACK, 350.0);
            encoderPositions.put(Position.SCORE, 4400.0);
        }
    }

    public static final class FlippableDualClawConfigs {
        public static final double flipperIntakePosition = 0.15, flipperHoldPosition = 0.8;

        public static final double leftClawClosePosition = 0.32, leftClawOpenPosition = 0.55, rightClawClosedPosition = 0.75, rightClawOpenPosition = 0.55;
    }

    public static final class ArmConfigsLegacy {
        public static final String armMotor1Name = "arm1", armMotor2Name = "arm2", armEncoderName = "arm1", limitSwitchName = "limit";
        public static final boolean armMotor1Reversed = true, armMotor2Reversed = false, armEncoderReversed = false;
        public static final double armMotorMaximumPower = 1;

        public static final int positionDifferenceStartDecelerate = 500;
        public static final int positionTolerance = 40;
        public static final double frictionPower = 0.25;
        public static final int feedPos = 0, lowPos = 2100, midPos = 2300, highPos = 3500;
        public static final int positionLimit = 3500;
        public static final ArmLegacy.ArmCommand armCommandWhenNoInput = new ArmLegacy.ArmCommand(ArmLegacy.ArmCommand.ArmCommandType.SET_MOTOR_POWER, 0); // null for not needed

//        public static final String claw1Name = "claw1", claw2Name = "claw2"; // dual servo claw
//        public static final Claw.ServoProfile claw1Profile = new Claw.ServoProfile(1-0.4, 1-0.16),
//                claw2Profile = new Claw.ServoProfile(0.4, 0.16);
        public static final Claw.ServoProfile claw1Profile = new Claw.ServoProfile(1, 0.8),
        claw2Profile = null;
        public static final String claw1Name = "claw1", claw2Name = null; // single servo claw

        public static final double servoValueOrigin = 0.95, servoValueExtend = 0.5;
        public static final double extendTime = 0.3;
        public static final int positionErrorAsCommandFinished = 80;
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
                new double[] {0, 0.08, 0.4, 0.8, 1} // motor power
        );


        public static final double chassisRotation_maximumCorrectionPower = 0.65;
        public static final double chassisRotation_minimumCorrectionPower = 0.03;
        public static final double chassisRotation_errorStartDecelerateRadian = Math.toRadians(45);
        public static final double chassisRotation_errorTolerance = Math.toRadians(1.5);
        public static final double chassisRotationErrorAsFinished = Math.toRadians(4);
        public static final double chassisRotationErrorAsRoughlyFinished = Math.toRadians(12);
        public static final double chassisRotationSpeedAsStopped = Math.toRadians(15); // rad/sec
        public static final double chassisRotation_feedForwardDelay = 0.15; // in seconds
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
                0.06, // for precise wall aiming
                40,
                1,
                0.14,
                0, 0
        );
        public static final EnhancedPIDController.StaticPIDProfile encoderTranslationalControllerProfileY = new EnhancedPIDController.StaticPIDProfile(
                Double.POSITIVE_INFINITY,
                1.05,
                0.05,
                40,
                1,
                0.13,
                0, 0
        );

        /** the maximum chassis speed when using visual navigation */
        public static final double lowSpeedModeMaximumMotorSpeedConstrain = 0.8;
        public static final double lowSpeedModeStickSensitivity = 0.6;

        /** a power constrain to the motors to not push them too hard */
        public static final double ordinaryModeMaximumMotorSpeedConstrain = 1.4;

        public static final double xPowerRate = 1.05,
                yPowerRate = 1;
        /** the distance to set the position target, when pilot sends a command of full speed  */
        public static final double targetDistanceAtMaxDesiredSpeed = 75;
        /** the smooth-out time, or the time after the pilot let the chassis stop till it start maintaining its current position */
        public static final double timeToStartDecelerateTranslation = 0.5;

        public static final double timeToStartDecelerateRotation = 0.3;

        /** within how many times the error tolerance of PID should the translational task be counted as complete */
        public static final double errorAsTaskFinishedCM = 1.5;
        public static final double chassisSpeedAsRobotStoppedCMPerSec = 5;

        public static final double errorAsTaskRoughlyFinished = 12;

        public static final double autoStageMaxVelocity = 135;
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
        public static final double errorTolerance = 1.5;

        public static final double distanceSensorMaxDistance = 45;
        public static final double distanceSensorMaxDistance_maintainAndAim = 15;
        public static final double distanceSensorMinDistance = 1;
        public static final double approachReverseSpeedTolerance = 20; // if the robot goes away from the wall in more than 20cm/s, we think the distance sensor failed

        public static final Vector2D targetedRelativePositionToWallRoughApproach = new Vector2D(new double[]{0, -40});
        public static final Vector2D targetedRelativePositionToWallPreciseTOFApproach = new Vector2D(new double[] {0, -3});
        public static final double[] aimHorizontalPositions = new double[] {0, 4, 12, 20};
        public static final double autoStageScoringHorizontalDeviation = 12;
        public static final double maximumXBiasToWallCenterDuringAimingCM = 20;
        public static final double approachPathSmoothOutPercent = 0.6;
        public static final double visualApproachSpeed = 150;
        public static final boolean faceToTargetWhenApproaching = true;

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

    public static final class TeamElementFinderConfigs {
        public static final Map<TeamElementFinder.TeamElementPosition, Double[]> teamElementPositionSearchRotationRanges = new HashMap<>();
        public static final double distanceThreshold = 20;
        static {
            teamElementPositionSearchRotationRanges.put(TeamElementFinder.TeamElementPosition.LEFT, new Double[]{Math.toRadians(125), Math.toRadians(55)});
            teamElementPositionSearchRotationRanges.put(TeamElementFinder.TeamElementPosition.CENTER, new Double[]{Math.toRadians(35), Math.toRadians(-35)});
            teamElementPositionSearchRotationRanges.put(TeamElementFinder.TeamElementPosition.RIGHT, new Double[]{Math.toRadians(-55), Math.toRadians(-125)});
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
        public static PilotChassisService.ControlMode defaultControlMode = PilotChassisService.ControlMode.ENCODER_ASSISTED_FIELD_ORIENTATED;

        public static final int pilotControllerKeyUpdatingRate = 60; // times per second
        public static final ControllerStick translationalControllerStick = ControllerStick.LEFT_HAND;
        public enum ControllerStick {
            RIGHT_HAND,
            LEFT_HAND
        }
    }

    public static final class KeyBindings {
        public static final XboxControllerKey resetIMUKey = XboxControllerKey.Y;
        public static final XboxControllerKey turnOffRotationControlButton = XboxControllerKey.YIELD;
        public static final XboxControllerKey facePilotLeftButton = XboxControllerKey.YIELD;
        public static final XboxControllerKey facePilotRightButton = XboxControllerKey.YIELD;
        public static final XboxControllerKey facePilotFrontButton = XboxControllerKey.YIELD;
        public static final XboxControllerKey facePilotBackButton = XboxControllerKey.YIELD;

        public static final XboxControllerKey moveLeftSlowlyButton = XboxControllerKey.DPAD_LEFT;
        public static final XboxControllerKey moveRightSlowlyButton = XboxControllerKey.DPAD_RIGHT;
        public static final XboxControllerKey moveForwardSlowlyButton = XboxControllerKey.DPAD_UP;
        public static final XboxControllerKey moveBackSlowlyButton = XboxControllerKey.DPAD_DOWN;
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
