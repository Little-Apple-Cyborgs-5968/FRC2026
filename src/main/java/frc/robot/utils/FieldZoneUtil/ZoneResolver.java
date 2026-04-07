package frc.robot.utils.FieldZoneUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.FieldConstants;
import java.util.List;

public class ZoneResolver {
    private final List<FieldZones.FieldZone> orderedZones;
    private final double robotLengthMeters;
    private final double robotWidthMeters;

    public ZoneResolver(
            List<FieldZones.FieldZone> orderedZones,
            double robotLengthMeters,
            double robotWidthMeters) {
        this.orderedZones = orderedZones;
        this.robotLengthMeters = robotLengthMeters;
        this.robotWidthMeters = robotWidthMeters;
    }

    public FieldZones.ZoneType getZone(Pose2d robotPose) {
        Pose2d bluePose = FieldConstants.flipIfRed(robotPose);

        Translation2d[] footprint = ZoneGeometry.robotFootprint(
            bluePose,
            robotLengthMeters,
            robotWidthMeters
        );

        for (FieldZones.FieldZone zone : orderedZones) {
            if (zone.shape().overlaps(footprint)) {
                return zone.type();
            }
        }

        return FieldZones.ZoneType.NONE;
    }

    public boolean isInZone(Pose2d robotPose, FieldZones.ZoneType zoneType) {
        return getZone(robotPose) == zoneType;
    }
}