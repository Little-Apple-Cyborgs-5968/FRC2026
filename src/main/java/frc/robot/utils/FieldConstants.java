package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {
    /** Full length of the FRC field in meters (2026 season) */
    public static final double kFieldLength = 16.5405;

    public static final Pose3d hubTarget = new Pose3d(4.620, 4.040, 3.057144, new Rotation3d());
    public static final Pose3d leftPassTarget = new Pose3d(2.50, 6.0, 0, new Rotation3d());
    public static final Pose3d rightPassTarget = new Pose3d(2.50, 1.960, 0, new Rotation3d());

    //Pose 2D (Blue alliance origin)
    public static final Pose2d leftTrenchShoot  = new Pose2d(3.650, 7.411, new Rotation2d(Math.PI));
    public static final Pose2d rightTrenchShoot = new Pose2d(3.650, 0.635, new Rotation2d(0));

    /**
     * Mirrors a Pose2d to the red alliance side of the field.
     * Flips X across the field center and rotates heading by 180°.
     */
    public static Pose2d flipPose(Pose2d pose) {
        return new Pose2d(
            kFieldLength - pose.getX(),
            pose.getY(),
            pose.getRotation().rotateBy(Rotation2d.k180deg)
        );
    }

    /**
     * Returns the pose flipped to the red alliance side if currently on red,
     * otherwise returns it unchanged.
     */
    public static Pose2d flipIfRed(Pose2d pose) {
        boolean isRed = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        return isRed ? flipPose(pose) : pose;
    }

    /** Left trench shoot pose, automatically flipped for red alliance */
    public static Pose2d getLeftTrenchShoot() {
        return flipIfRed(leftTrenchShoot);
    }

    /** Right trench shoot pose, automatically flipped for red alliance */
    public static Pose2d getRightTrenchShoot() {
        return flipIfRed(rightTrenchShoot);
    }
}

