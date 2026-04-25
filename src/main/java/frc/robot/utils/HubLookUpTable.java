// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.TreeMap;

/** 
 * Lookup table for hub shooting parameters based on distance.
 * Supports linear interpolation between data points.
 */
public class HubLookUpTable {
    
    /** Multiplier applied to all time of flight values because our time of flight measures is a bum aids monkey */
    private static final double TIME_OF_FLIGHT_MULTIPLIER = 0.8;
    
    /** Data structure to hold shooting parameters */
    public static class ShootingParameters {
        public final double shooterSpeed;     // RPS (Revolutions Per Second)
        public final double trajectoryAngle;  // Degrees
        public final double timeOfFlight;     // Seconds
        
        public ShootingParameters(double shooterSpeed, double trajectoryAngle, double timeOfFlight) {
            this.shooterSpeed = shooterSpeed;
            this.trajectoryAngle = trajectoryAngle;
            this.timeOfFlight = timeOfFlight;
        }
    }
    
    // TreeMap automatically sorts by distance (key)
    private final TreeMap<Double, ShootingParameters> lookupTable;
    
    public HubLookUpTable() {
        lookupTable = new TreeMap<>();
        initializeLookupTable();
    }
    
    /** Initialize the lookup table with known data points */
    private void initializeLookupTable() {
        // Distance (m), Shooter Speed (RPS), Trajectory Angle (°), Time of Flight (s)
        // KrakenX60 shooting 226g ball - optimized for constant RPS ~75
        // Trajectory angles: 90° = straight up, 45° = maximum distance
        addEntry(0.84,10.5,79,0.94 * TIME_OF_FLIGHT_MULTIPLIER);  // Close shot - nearly straight up
        addEntry(1.04,10.8,76,0.84 * TIME_OF_FLIGHT_MULTIPLIER);
        addEntry(2.37,11.95,65.5,1.02 * TIME_OF_FLIGHT_MULTIPLIER);
        addEntry(2.86,12.1,65,1.0 * TIME_OF_FLIGHT_MULTIPLIER);
        
        addEntry(3.6,13.25,56,1.04 * TIME_OF_FLIGHT_MULTIPLIER);
        addEntry(4.45,14.56,55,1.14 * TIME_OF_FLIGHT_MULTIPLIER);
        addEntry(4.93,16.01,51,1.22 * TIME_OF_FLIGHT_MULTIPLIER);
        addEntry(5.8,16.5,51,1.05 * TIME_OF_FLIGHT_MULTIPLIER);
        addEntry(8,17.0,45,1.05 * TIME_OF_FLIGHT_MULTIPLIER);
        // Max distance - lowest angle
    }
    
    /** Add an entry to the lookup table */
    public void addEntry(double distance, double shooterSpeed, double trajectoryAngle, double timeOfFlight) {
        lookupTable.put(distance, new ShootingParameters(shooterSpeed, trajectoryAngle, timeOfFlight));
    }
    
    /** 
     * Get interpolated shooting parameters for a given distance 
     * @param distance Distance to target in meters
     * @return Interpolated shooting parameters
     */
    public ShootingParameters getParameters(double distance) {
        // Check if exact match exists
        if (lookupTable.containsKey(distance)) {
            return lookupTable.get(distance);
        }
        
        // Get the surrounding values
        Double lowerKey = lookupTable.floorKey(distance);
        Double upperKey = lookupTable.ceilingKey(distance);
        
        // Handle edge cases
        if (lowerKey == null) {
            return lookupTable.get(upperKey); // Below minimum distance
        }
        if (upperKey == null) {
            return lookupTable.get(lowerKey); // Above maximum distance
        }
        
        // Perform linear interpolation
        ShootingParameters lower = lookupTable.get(lowerKey);
        ShootingParameters upper = lookupTable.get(upperKey);
        
        double ratio = (distance - lowerKey) / (upperKey - lowerKey);
        
        double interpolatedSpeed = lerp(lower.shooterSpeed, upper.shooterSpeed, ratio);
        double interpolatedAngle = lerp(lower.trajectoryAngle, upper.trajectoryAngle, ratio);
        double interpolatedTime = lerp(lower.timeOfFlight, upper.timeOfFlight, ratio);
        
        return new ShootingParameters(interpolatedSpeed, interpolatedAngle, interpolatedTime);
    }
    
    /** Linear interpolation helper */
    private double lerp(double start, double end, double ratio) {
        return start + (end - start) * ratio;
    }
    
    /** Get shooter speed for a given distance */
    public double getShooterSpeed(double distance) {
        return getParameters(distance).shooterSpeed;
    }
    
    /** Get trajectory angle for a given distance */
    public double getTrajectoryAngle(double distance) {
        return getParameters(distance).trajectoryAngle;
    }
    
    /** Get time of flight for a given distance */
    public double getTimeOfFlight(double distance) {
        return getParameters(distance).timeOfFlight;
    }
}
