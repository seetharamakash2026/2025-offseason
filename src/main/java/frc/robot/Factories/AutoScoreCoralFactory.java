package frc.robot.Factories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.ElevatorPivot;
import static frc.robot.Constants.AutoScoreConstants.*;

public class AutoScoreCoralFactory {
    private static CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
    private static Claw claw = RobotContainer.claw;
    private static ElevatorPivot elevatorPivot = RobotContainer.elevatorPivot;
    private static IntegerSubscriber poleHeightSubscriber = RobotContainer.poleHeightSubscriber;

    private Translation2d midpoint(Translation2d a, Translation2d b) {
        return new Translation2d(a.getX() + (b.getX() - a.getX())/2, a.getY() + (b.getY() - a.getY())/2);
    }

    private Pose2d getRobotPose() {
        return drivetrain.getState().Pose;
    }

    public Pose2d getAutoscorePose() {
        Translation2d[] midpoints;

        //Alliance
        if (DriverStation.getAlliance().get() == Alliance.Red) {
        midpoints = new Translation2d[]{
            midpoint(POLEPOSESRED[0], POLEPOSESRED[1]),
            midpoint(POLEPOSESRED[2], POLEPOSESRED[3]),
            midpoint(POLEPOSESRED[4], POLEPOSESRED[5]),
            midpoint(POLEPOSESRED[6], POLEPOSESRED[7]),
            midpoint(POLEPOSESRED[8], POLEPOSESRED[9]),
            midpoint(POLEPOSESRED[10], POLEPOSESRED[11])
        };
        } else {
        midpoints = new Translation2d[]{
            midpoint(POLEPOSESBLUE[0], POLEPOSESBLUE[1]),
            midpoint(POLEPOSESBLUE[2], POLEPOSESBLUE[3]),
            midpoint(POLEPOSESBLUE[4], POLEPOSESBLUE[5]),
            midpoint(POLEPOSESBLUE[6], POLEPOSESBLUE[7]),
            midpoint(POLEPOSESBLUE[8], POLEPOSESBLUE[9]),
            midpoint(POLEPOSESBLUE[10], POLEPOSESBLUE[11])
        };
        }
        Translation2d closest = midpoints[0];
        int idx = 0;
        for (int i = 0; i < midpoints.length; i ++) {
        if (getRobotPose().getTranslation().getDistance(midpoints[i]) < getRobotPose().getTranslation().getDistance(closest)) {
            closest = midpoints[i];
            idx = i;
        }
        }
        return new Pose2d(closest, new Rotation2d(ROTATIONS[idx]));
    }
}
