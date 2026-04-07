package frc.robot.utils.FieldZoneUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class ZoneGeometry {
    private ZoneGeometry() {}

    public interface ZoneShape {
        boolean containsPoint(Translation2d point);
        boolean overlaps(Translation2d[] polygon);
        Translation2d[] getVertices();
    }

    public static class ConvexPolygonZone implements ZoneShape {
        private final Translation2d[] vertices;

        public ConvexPolygonZone(Translation2d... vertices) {
            this.vertices = vertices.clone();
        }

        @Override
        public Translation2d[] getVertices() {
            return vertices.clone();
        }

        @Override
        public boolean containsPoint(Translation2d p) {
            int n = vertices.length;
            double prevCross = 0.0;

            for (int i = 0; i < n; i++) {
                Translation2d a = vertices[i];
                Translation2d b = vertices[(i + 1) % n];

                double edgeX = b.getX() - a.getX();
                double edgeY = b.getY() - a.getY();
                double pointX = p.getX() - a.getX();
                double pointY = p.getY() - a.getY();

                double cross = edgeX * pointY - edgeY * pointX;

                if (Math.abs(cross) < 1e-9) {
                    continue;
                }

                if (prevCross == 0.0) {
                    prevCross = cross;
                } else if (cross * prevCross < 0) {
                    return false;
                }
            }

            return true;
        }

        @Override
        public boolean overlaps(Translation2d[] other) {
            return !hasSeparatingAxis(this.vertices, other)
                && !hasSeparatingAxis(other, this.vertices);
        }

        private static boolean hasSeparatingAxis(Translation2d[] polyA, Translation2d[] polyB) {
            for (int i = 0; i < polyA.length; i++) {
                Translation2d p1 = polyA[i];
                Translation2d p2 = polyA[(i + 1) % polyA.length];

                double axisX = -(p2.getY() - p1.getY());
                double axisY =  (p2.getX() - p1.getX());

                double[] aProj = project(polyA, axisX, axisY);
                double[] bProj = project(polyB, axisX, axisY);

                if (aProj[1] < bProj[0] || bProj[1] < aProj[0]) {
                    return true;
                }
            }
            return false;
        }

        private static double[] project(Translation2d[] poly, double axisX, double axisY) {
            double min = dot(poly[0], axisX, axisY);
            double max = min;

            for (int i = 1; i < poly.length; i++) {
                double val = dot(poly[i], axisX, axisY);
                min = Math.min(min, val);
                max = Math.max(max, val);
            }

            return new double[] {min, max};
        }

        private static double dot(Translation2d p, double axisX, double axisY) {
            return p.getX() * axisX + p.getY() * axisY;
        }
    }

    public static class TriangleZone extends ConvexPolygonZone {
        public TriangleZone(Translation2d a, Translation2d b, Translation2d c) {
            super(a, b, c);
        }
    }

    public static class RectangleZone extends ConvexPolygonZone {
        public RectangleZone(
                Translation2d bottomLeft,
                Translation2d bottomRight,
                Translation2d topRight,
                Translation2d topLeft) {
            super(bottomLeft, bottomRight, topRight, topLeft);
        }

        public static RectangleZone axisAligned(double xMin, double xMax, double yMin, double yMax) {
            return new RectangleZone(
                new Translation2d(xMin, yMin),
                new Translation2d(xMax, yMin),
                new Translation2d(xMax, yMax),
                new Translation2d(xMin, yMax)
            );
        }
    }


    
    public static class CompositeZone implements ZoneShape {
    private final ZoneShape[] parts;

    public CompositeZone(ZoneShape... parts) {
        this.parts = parts.clone();
    }

    @Override
    public boolean containsPoint(Translation2d point) {
        for (ZoneShape z : parts) {
            if (z != null && z.containsPoint(point)) {
                return true;
            }
        }
        return false;
    }

    @Override
    public boolean overlaps(Translation2d[] polygon) {
        for (ZoneShape z : parts) {
            if (z != null && z.overlaps(polygon)) {
                return true;
            }
        }
        return false;
    }



    @Override
    public Translation2d[] getVertices() {
        // concatenate vertices for diagnostics/visualization (order not important)
        int total = 0;
        for (ZoneShape z : parts) if (z != null) total += z.getVertices().length;
        Translation2d[] out = new Translation2d[total];
        int i = 0;
        for (ZoneShape z : parts) {
            if (z == null) continue;
            for (Translation2d v : z.getVertices()) out[i++] = v;
        }
        return out;
    }
}

    public static Translation2d[] robotFootprint(
            Pose2d pose,
            double robotLengthMeters,
            double robotWidthMeters) {

        double halfL = robotLengthMeters / 2.0;
        double halfW = robotWidthMeters / 2.0;

        Translation2d[] localCorners = new Translation2d[] {
            new Translation2d( halfL,  halfW),
            new Translation2d( halfL, -halfW),
            new Translation2d(-halfL, -halfW),
            new Translation2d(-halfL,  halfW)
        };

        Translation2d center = pose.getTranslation();
        Rotation2d rot = pose.getRotation();

        Translation2d[] worldCorners = new Translation2d[4];
        for (int i = 0; i < 4; i++) {
            worldCorners[i] = center.plus(localCorners[i].rotateBy(rot));
        }

        return worldCorners;
    }
}