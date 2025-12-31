package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static class Swerve{
        public static final double kRobotCentricSpeed = 0.5; //how fast it robot centric drive speed is in m/s
    }
    public static class PathFinding{

        // Default path constraints for when pathfinding
        public static final PathConstraints kDefualtConstraints = new PathConstraints(
            4.0, 3.0,  // max velocity, max acceleration (m/s, m/s²)
            Units.degreesToRadians(540), Units.degreesToRadians(720)  // max angular vel, max angular accel
        );
    }
}
