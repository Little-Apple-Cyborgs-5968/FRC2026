package frc.robot.utils.FieldZoneUtil;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.FieldZoneUtil.ZoneGeometry;

import java.util.List;

public final class FieldZones {
    private FieldZones() {}

    public enum ZoneType {
        TRENCH,
        BLOCKED,
        ALLIANCE,
        NEUTRAL,
        OPP,
        NONE
    }

    public enum TrackingType {
        BUMPER, // Determined by robot bumper/footprint overlap
        TURRET  // Determined by turret Pose2d (center point) containment
    }

    public record FieldZone(ZoneType type, ZoneGeometry.ZoneShape shape, TrackingType trackingType) {}

    // everything in meters

    public static final FieldZone TRENCH = new FieldZone(
        ZoneType.TRENCH,
        new ZoneGeometry.CompositeZone(
            // close left trench piece
            ZoneGeometry.RectangleZone.axisAligned(4.028,5.218,6.402,8.07),
            // close right trench piece
            ZoneGeometry.RectangleZone.axisAligned(4.028,5.218,0,1.668),
            // far left trench piece
            ZoneGeometry.RectangleZone.axisAligned(11.322,12.512,6.402,8.07),
            // fra right trench piece
            ZoneGeometry.RectangleZone.axisAligned(11.322,12.512,0,1.668)
        ),
        TrackingType.TURRET
    );

    public static final FieldZone ALLIANCE = new FieldZone(
        ZoneType.ALLIANCE,
        ZoneGeometry.RectangleZone.axisAligned(0,4.028,0,8.07),
        TrackingType.BUMPER
    );

    public static final FieldZone NEUTRAL = new FieldZone(
        ZoneType.NEUTRAL,
        ZoneGeometry.RectangleZone.axisAligned(5.218,11.322,0,8.07),
        TrackingType.BUMPER
    );

    public static final FieldZone OPP = new FieldZone(
        ZoneType.OPP,
        ZoneGeometry.RectangleZone.axisAligned(12.54,16.54,0,8.07),
        TrackingType.BUMPER
    );  

    public static final FieldZone BLOCKED = new FieldZone(
        ZoneType.BLOCKED,
        new ZoneGeometry.CompositeZone(
            new ZoneGeometry.TriangleZone(
                new Translation2d(5.218,3.346),
                new Translation2d(5.218,4.724),
                new Translation2d(6.9   ,4.035)
            ),
            new ZoneGeometry.TriangleZone(
                new Translation2d(12.512,3.346),
                new Translation2d(12.512,4.724),
                new Translation2d(16.5,4.035)
            )
        ),
        TrackingType.TURRET
    );

    public static final List<FieldZone> ORDERED_ZONES = List.of(
        BLOCKED,
        OPP,
        NEUTRAL,
        ALLIANCE,
        TRENCH
        
        
        
    );
}