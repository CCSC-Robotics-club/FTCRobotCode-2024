package org.firstinspires.ftc.teamcode.Utils;

/**
 * a simple velocity controller
 * TODO test this controller and implement it in the code
 */
public class SimpleFeedForwardSpeedController {
    private static final double nanoToSec = 1_000_000_000.0;
    private final double proportionGain, feedForwardGain, feedForwardDelay;
    private double previousSpeed;
    private double previousTimeNano;
    /**
     *  initializes the speed controller with parameters
     * @param profile the profile for the speed controller
     *  */
    public SimpleFeedForwardSpeedController(SpeedControllerProfile profile) {
        this.proportionGain = profile.proportionGain;
        this.feedForwardGain = profile.feedForwardGain;
        this.feedForwardDelay = profile.feedForwardDelay;
    }

    public double getSpeedControlPower(double currentSpeed, double desiredSpeed) {
        double basePower = desiredSpeed * proportionGain;
        double dt = ((double) (System.nanoTime() - previousTimeNano)) / nanoToSec;
        double currentAcceleration = (currentSpeed - previousSpeed) / dt;
        double predictedSpeed = currentSpeed + currentAcceleration * feedForwardDelay;
        double feedForwardPower = (desiredSpeed - predictedSpeed) * feedForwardGain;

        this.previousSpeed = currentSpeed;
        this.previousTimeNano = System.nanoTime();

        return feedForwardPower + basePower;
    }

    public static class SpeedControllerProfile{
        public final double proportionGain, feedForwardGain, feedForwardDelay;
        public SpeedControllerProfile(double proportionGain, double feedForwardGain, double feedForwardDelay) {
            this.proportionGain = proportionGain;
            this.feedForwardGain = feedForwardGain;
            this.feedForwardDelay = feedForwardDelay;
        }
    }
}
