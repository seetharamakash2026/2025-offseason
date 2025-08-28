package frc.robot.Factories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Commands.DriveToPose;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.ElevatorPivot;
import frc.robot.commons.Translation2DUtils;

import static frc.robot.Constants.AutoScoreConstants.*;

public class AutoScoreCoralFactory {
    private static CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
    private static Claw claw = RobotContainer.claw;
    private static ElevatorPivot elevatorPivot = RobotContainer.elevatorPivot;
    private static IntegerSubscriber poleHeightSubscriber = RobotContainer.poleHeightSubscriber;

    private Translation2d midpoint(Translation2d a, Translation2d b) {
        return new Translation2d(a.getX() + (b.getX() - a.getX())/2, a.getY() + (b.getY() - a.getY())/2);
    }

    private Command pathfindWithOffset(Pose2d pos, Translation2d offset) {
        return new DriveToPose(drivetrain, new Pose2d(pos.getTranslation().minus(offset), pos.getRotation()));
    }

    private Command pathfindDistance(Pose2d pos, double dist) {
        Translation2d reefCenter;
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                reefCenter = REEF_CENTERS[0];
            } else {
                reefCenter = REEF_CENTERS[1];
            }
        } else { //Default to red
            reefCenter = REEF_CENTERS[0];
        }

        Translation2d vector = reefCenter.minus(pos.getTranslation());
        vector = Translation2DUtils.normalize(vector);
        vector.times(dist);
        return pathfindWithOffset(pos, vector);
    }

    private Pose2d getRobotPose() {
        return drivetrain.getState().Pose;
    }

    public Pose2d getAutoscorePose() {
        Translation2d[] midpoints;

        //Alliance
        if (DriverStation.getAlliance().isPresent()) {
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
        } else { //Default to red
            midpoints = new Translation2d[]{
                midpoint(POLEPOSESRED[0], POLEPOSESRED[1]),
                midpoint(POLEPOSESRED[2], POLEPOSESRED[3]),
                midpoint(POLEPOSESRED[4], POLEPOSESRED[5]),
                midpoint(POLEPOSESRED[6], POLEPOSESRED[7]),
                midpoint(POLEPOSESRED[8], POLEPOSESRED[9]),
                midpoint(POLEPOSESRED[10], POLEPOSESRED[11])
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

    public Command goToScoreHeightAndDriveForwad() {
        // long poleNum = poleHeightSubscriber.get();
        long poleNum = 1;
        double height = SCORE_HEIGHTS[(int) poleNum];
        double rot = SCORE_ANGLES[(int) poleNum];
        return elevatorPivot.goToPosition(() -> {return height;}, () -> {return rot;}).andThen(pathfindDistance(getRobotPose(), FRONT_DIST));
    }

    private Command stow() {
        return elevatorPivot.stowArm();
    }

    public Command reset() {
        return pathfindDistance(getRobotPose(), BACKUP_DIST).andThen(stow());
    }

    public Command goToScorePos() {
        return new DriveToPose(drivetrain, getAutoscorePose());
    }

    public Command score() {
        return claw.outtakeGeneric();
    }

    public Command fullAutoscore() {
        return goToScorePos().andThen(goToScoreHeightAndDriveForwad()).andThen(score()).andThen(reset());
    }
}
