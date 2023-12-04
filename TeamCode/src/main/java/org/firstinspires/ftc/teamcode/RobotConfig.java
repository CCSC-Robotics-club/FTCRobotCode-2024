package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Modules.Arm;
import org.firstinspires.ftc.teamcode.Modules.FixedAnglePixelCamera;
import org.firstinspires.ftc.teamcode.Modules.TripleIndependentEncoderAndIMUPositionEstimator;
import org.firstinspires.ftc.teamcode.Services.PilotChassisService;
import org.firstinspires.ftc.teamcode.Utils.Claw;
import org.firstinspires.ftc.teamcode.Utils.FixedAngleCameraProfile;
import org.firstinspires.ftc.teamcode.Utils.EnhancedPIDController;
import org.firstinspires.ftc.teamcode.Utils.Vector2D;

public final class RobotConfig {
    public static final HardwareConfigs hardwareConfigs_2024Competition_backup = new HardwareConfigs( // backup machine
            new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            ),
//            new IMU.Parameters(
//                    new RevHubOrientationOnRobot(
//                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                            RevHubOrientationOnRobot.UsbFacingDirection.UP
//                    )
//            ),
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
//            new IMU.Parameters(
//                    new RevHubOrientationOnRobot(
//                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                            RevHubOrientationOnRobot.UsbFacingDirection.UP
//                    )
//            ),
            new IMU.Parameters(
                    new RevHubOrientationOnRobot( // expansion hub
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
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

    public static final HardwareConfigs testConfig = hardwareConfigs_2024Competition;

    public static final class IntakeConfigs {
        public static final String intakeMotor1Name = "intake1", intakeMotor2Name = "intake2";
        public static final double intakeMotor1Power = -0.75, intakeMotor2Power = 0.75; // competition machine
//        public static final double intakeMotor1Power = -0.85, intakeMotor2Power = -0.85; // backup machine
    }

    public static final class ArmConfigs {
        public static final String armMotorName = "arm", armEncoderName = "arm", limitSwitchName = "limit";
        public static final boolean armMotorReversed = true, armEncoderReversed = false;
        public static final double armMotorMaximumPower = 0.75;

        public static final int positionDifferenceStartDecelerate = 500;
        public static final int positionTolerance = 50;
        public static final double frictionPower = 0.2;
        public static final int lowPos = 1800, midPos = 2600, highPos = 3400;
        public static final int positionLimit = 3500;
        public static final Arm.ArmCommand armCommandWhenNoInput = new Arm.ArmCommand(Arm.ArmCommand.ArmCommandType.SET_MOTOR_POWER, 0); // null for not needed

//        public static final String claw1Name = "claw1", claw2Name = "claw2"; // dual servo claw
//        public static final Claw.ServoProfile claw1Profile = new Claw.ServoProfile(1-0.4, 1-0.16),
//                claw2Profile = new Claw.ServoProfile(0.4, 0.16);
        public static final Claw.ServoProfile claw1Profile = new Claw.ServoProfile(1, 0.8),
        claw2Profile = null;
        public static final String claw1Name = "claw1", claw2Name = null; // single servo claw
    }

    public static final class ChassisConfigs {
        /*
         * to tune the pid of these wheels:
         * 1. find the wheel speed (in encoder units per second) of the wheel when running at full speed as "max_speed" (2500)
         * 2. find the wheel speed (in encoder units per second) of the wheel when running at 65% full speed as "speed_65" (2000)
         */
        public static final boolean wheelSpeedControlEnabledDefault = false;
        public static final double wheel_proportionGain = 0.8;
        public static final double wheel_feedForwardGain = 0.55;
        public static final double wheel_feedForwardDelay = 0;
        public static final double wheel_maxVelocity = 2500;


        public static final double chassisRotation_maximumCorrectionPower = 0.6;
        public static final double chassisRotation_minimumCorrectionPower = 0.06;
        public static final double chassisRotation_errorStartDecelerateRadian = Math.toRadians(40);
        public static final double chassisRotation_errorTolerance = Math.toRadians(1.5);
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
        public static final int positionEstimator_speedEstimationFrequency = 40;

        /* encoder drive-to-position PID */
        public static final EnhancedPIDController.StaticPIDProfile encoderTranslationalControllerProfile = new EnhancedPIDController.StaticPIDProfile(
                Double.POSITIVE_INFINITY,
                0.95,
                0.05,
                40,
                1,
                0.13,
                0, 0
        );

        /** the maximum chassis speed when using visual navigation */
        public static final double lowSpeedModeMaximumMotorSpeedConstrain = 0.6;

        /** a power constrain to the motors to not push them too hard */
        public static final double ordinaryModeMaximumMotorSpeedConstrain = 1.2;

        public static final double xPowerRate = 1.05,
                yPowerRate = 1;
        /** the distance to set the position target, when pilot sends a command of full speed  */
        public static final double targetDistanceAtMaxDesiredSpeed = 120;
        /** the smooth-out time, or the time after the pilot let the chassis stop till it start maintaining its current position */
        public static final double timeToStartDecelerate = 0.4;

        /** within how many times the error tolerance of PID should the translational task be counted as complete */
        public static final double errorToleranceAsTaskFinished = 1.8;

        public static final double errorToleranceAsTaskRoughlyFinished = 8;

        public static final double autoStageMaxVelocity = 300;
        public static final double autoStageMaxAcceleration = 350;
        public static final double autoStageMaxAngularVelocity = Math.PI * 1.4;
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
                0.955,
                -0.00315,
                -0.003432,
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

        public static final double distanceSensorMaxDistance = 40;
        public static final double distanceSensorMaxDistance_maintainAndAim = 15;
        public static final double distanceSensorMinDistance = 1.5;
        public static final double approachReverseSpeedTolerance = 20; // if the robot goes away from the wall in more than 20cm/s, we think the distance sensor failed

        public static final Vector2D targetedRelativePositionToWallRoughApproach = new Vector2D(new double[]{0, -23});
        public static final Vector2D targetedRelativePositionToWallPreciseTOFApproach = new Vector2D(new double[] {0, -3.5});
        public static final double[] aimHorizontalPositions = new double[] {0, 4, 12, 20};
        public static final double maximumXBiasToWallCenterDuringAimingCM = 22;
        public static final double approachPathSmoothOutPercent = 0.6;
        public static final double visualApproachSpeed = 100;
        public static final boolean faceToTargetWhenApproaching = true;

        public static final long maxTimeToWaitForVisualNavigationMS = 800; // after this much milliseconds, if the target still does not occur, the navigation will be considered failed



        public static final FixedAngleCameraProfile pixelCameraSetUpProfile = new FixedAngleCameraProfile(
                20,
                0.70376,
                -0.002541,
                -0.001717,
                new double[2], new double[2]
        );
        public static final double pixelCameraInstallFacing = Math.PI;
        public static final double pixelDetectionMaximumDistance = 50; // in cm
        public static final double pixelSearchVelocity = 100; // in cm/s
        public static final Vector2D pixelFeedingSweetSpot = new Vector2D(new double[]{0, 15});
        public static final double feedingDistanceForward = -10; // in cm
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
        public static final double pilotController_rotationalStickExp = 1.8;


        public static final double visualNavigationAimingSensitivityCMPS = 30;
        public static final double triggerThreshold = 0.4;
        public static PilotChassisService.ControlMode defaultControlMode = PilotChassisService.ControlMode.MANUAL;

        public static final int pilotControllerKeyUpdatingRate = 60; // times per second
        public static final ControllerStick translationalControllerStick = ControllerStick.RIGHT_HAND;
        public enum ControllerStick {
            RIGHT_HAND,
            LEFT_HAND
        }
    }

    public static final class KeyBindings {
        public static final XboxControllerKey resetIMUKey = XboxControllerKey.DPAD_RIGHT;
        public static final XboxControllerKey toggleSpeedControlButton = XboxControllerKey.A;
        public static final XboxControllerKey maintainCurrentRotationButton = XboxControllerKey.LEFT_STICK_BUTTON;
        public static final XboxControllerKey faceFrontButton = XboxControllerKey.DPAD_DOWN;
        public static final XboxControllerKey toggleChassisDriveModeButton = XboxControllerKey.DPAD_UP;
        public static final XboxControllerKey processVisualApproachButton = XboxControllerKey.LEFT_BUMPER;
        public static final XboxControllerKey setAimPositionLeftButton = XboxControllerKey.X;
        public static final XboxControllerKey setAimPositionRightButton = XboxControllerKey.B;

        public static final XboxControllerKey processAutoIntakePixelButton = XboxControllerKey.RIGHT_TRIGGER;
    }

    public enum XboxControllerKey {
        A, B, X, Y,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        LEFT_BUMPER, RIGHT_BUMPER,
        LEFT_TRIGGER, RIGHT_TRIGGER,
        LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON
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
