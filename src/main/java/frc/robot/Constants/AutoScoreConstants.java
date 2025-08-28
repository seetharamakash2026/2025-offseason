package frc.robot.Constants;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoScoreConstants {
    //Each pole in order
    public static Translation2d[] POLEPOSESRED = {
        new Translation2d(3.18, 4.18),
        new Translation2d(3.17, 3.850),
        new Translation2d(3.69, 2.91),
        new Translation2d(3.97, 2.76),
        new Translation2d(5.02, 2.76),
        new Translation2d(5.32, 2.94),
        new Translation2d(5.85, 3.87),
        new Translation2d(5.85, 4.21),
        new Translation2d(5.30, 5.1),
        new Translation2d(5.011, 5.26),
        new Translation2d(3.97, 5.26),
        new Translation2d(3.66, 5.08)
    };
    public static Translation2d[] POLEPOSESBLUE = {
        FlippingUtil.flipFieldPose(new Pose2d(POLEPOSESRED[0], Rotation2d.kZero)).getTranslation(),
        FlippingUtil.flipFieldPose(new Pose2d(POLEPOSESRED[1], Rotation2d.kZero)).getTranslation(),
        FlippingUtil.flipFieldPose(new Pose2d(POLEPOSESRED[2], Rotation2d.kZero)).getTranslation(),
        FlippingUtil.flipFieldPose(new Pose2d(POLEPOSESRED[3], Rotation2d.kZero)).getTranslation(),
        FlippingUtil.flipFieldPose(new Pose2d(POLEPOSESRED[4], Rotation2d.kZero)).getTranslation(),
        FlippingUtil.flipFieldPose(new Pose2d(POLEPOSESRED[5], Rotation2d.kZero)).getTranslation(),
        FlippingUtil.flipFieldPose(new Pose2d(POLEPOSESRED[6], Rotation2d.kZero)).getTranslation(),
        FlippingUtil.flipFieldPose(new Pose2d(POLEPOSESRED[7], Rotation2d.kZero)).getTranslation(),
        FlippingUtil.flipFieldPose(new Pose2d(POLEPOSESRED[8], Rotation2d.kZero)).getTranslation(),
        FlippingUtil.flipFieldPose(new Pose2d(POLEPOSESRED[9], Rotation2d.kZero)).getTranslation(),
        FlippingUtil.flipFieldPose(new Pose2d(POLEPOSESRED[10], Rotation2d.kZero)).getTranslation(),
        FlippingUtil.flipFieldPose(new Pose2d(POLEPOSESRED[11], Rotation2d.kZero)).getTranslation()
    };

    public static double[] ROTATIONS = {
        0,
        60,
        120,
        180,
        240,
        300
    };

    public static double[] SCORE_HEIGHTS = {
        1,
        3,
        5,
        7
    };

    public static double[] SCORE_ANGLES = {
        0,
        90,
        180,
        360
    };

    public static Translation2d[] REEF_CENTERS = {
        new Translation2d(10, 10),
        new Translation2d(-10, 10)
    };

    public static double BACKUP_DIST = 1;
    public static double FRONT_DIST = 1;
}
