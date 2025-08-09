package frc.robot.commons;

import edu.wpi.first.math.geometry.Translation2d;

public class Translation2DUtils {
    public static Translation2d normalize(Translation2d in) {
        return in.div(in.getDistance(new Translation2d(0, 0)));
    }
}
