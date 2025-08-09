package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;

public class AutoScoreConstants {
    //Each pole in order
    public static Translation2d[] POLEPOSESRED = {
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10)
    };
    public static Translation2d[] POLEPOSESBLUE = {
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10),
        new Translation2d(10, 10)
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
}
