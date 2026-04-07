package frc.robot.utils.FieldZoneUtil;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public final class FieldZones {
    private FieldZones() {}

    public enum ZoneType {
        TRENCH,
        BLOCKED,
        ALLIANCE,
        NEUTRAL,
        NONE
    }

    public record FieldZone(ZoneType type, ZoneGeometry.ZoneShape shape) {}

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
        )
    );

    public static final FieldZone ALLIANCE = new FieldZone(
        ZoneType.ALLIANCE,
        ZoneGeometry.RectangleZone.axisAligned( 0,4.028,0,8.07)
    );

    public static final FieldZone NEUTRAL = new FieldZone(
        ZoneType.NEUTRAL,
        ZoneGeometry.RectangleZone.axisAligned(5.218,11.322,0,8.07)
    );

    public static final FieldZone BLOCKED = new FieldZone(
        ZoneType.BLOCKED,
        new ZoneGeometry.TriangleZone(
            new Translation2d(0,0),
            new Translation2d(1,0),
            new Translation2d(0,1)
        )
    );

    public static final List<FieldZone> ORDERED_ZONES = List.of(
        ALLIANCE,
        BLOCKED,
        NEUTRAL,
        TRENCH
        
    );
}