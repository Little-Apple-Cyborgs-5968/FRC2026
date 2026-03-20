package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallCounter extends SubsystemBase {
    private final CANrange canRange;
    
    private int ballCount = 0;
    private boolean countingEnabled = true;
    
    // To implement the rising edge and cooldown
    private boolean wasBallPresent = false;
    private double lastCountTime = 0.0;
    
    public BallCounter() {
        canRange = new CANrange(Constants.BallCounter.kCanId);
        
        CANrangeConfiguration config = new CANrangeConfiguration();
        
        // Use the short range 100hz CANrange setting
        config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        
        canRange.getConfigurator().apply(config);
        canRange.getDistance().setUpdateFrequency(Constants.BallCounter.kUpdateFrequencyHz);
    }
    
    @Override
    public void periodic() {
        // Retrieve newly refreshed distance measurement in meters
        double currentDistance = getDistance();


        // Define if a ball is in the feeder based on distance 
        boolean isBallInFeeder = (Constants.BallCounter.kBallInFeederMinMeters < currentDistance  && currentDistance < Constants.BallCounter.kBallInFeederMaxMeters);
        boolean isBallPresent = isBallInFeeder && currentDistance < Constants.BallCounter.kDetectBallThresholdMeters;
        
        // Edge detection: true if ball was not present last tick, but is present now
        if (countingEnabled && isBallPresent && !wasBallPresent) {
            double currentTime = Timer.getFPGATimestamp();
            
            // Check cooldown period (50ms)
            if ((currentTime - lastCountTime) >= Constants.BallCounter.kCooldownSeconds) {
                ballCount++;
                lastCountTime = currentTime;
            }
        }
        
        // Remember state for next loop
        wasBallPresent = isBallPresent;
        
        // Publish stats to dashboard for visibility
        SmartDashboard.putNumber("BallCounter/DistanceMeters", currentDistance);
        SmartDashboard.putNumber("BallCounter/Count", ballCount);
        SmartDashboard.putBoolean("BallCounter/IsBallPresent", isBallPresent);
        SmartDashboard.putBoolean("BallCounter/CountingEnabled", countingEnabled);
    }
    
    /**
     * Resets the counted balls to 0.
     */
    public void resetBallCount() {
        ballCount = 0;
    }
    
    /**
     * Pauses the automatic counting of balls.
     */
    public void stopBallCount() {
        countingEnabled = false;
    }
    
    /**
     * Resumes the automatic counting of balls.
     */
    public void startBallCount() {
        countingEnabled = true;
    }
    
    /**
     * Retrieves the current sensed distance in meters.
     * Refresh ensures we are getting the latest signal from the CAN bus.
     * 
     * @return Hardware-measured distance to the closest object in meters.
     */
    public double getDistance() {
        return canRange.getDistance().refresh().getValueAsDouble();
    }
    
    /**
     * Retrieves the total count of passing balls recorded.
     * 
     * @return Integer number of counted balls.
     */
    public int getBallCount() {
        return ballCount;
    }
}
