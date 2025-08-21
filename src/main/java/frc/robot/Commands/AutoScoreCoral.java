// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.ElevatorPivot;
import frc.robot.commons.Translation2DUtils;

import static frc.robot.Constants.AutoScoreConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScoreCoral extends Command {
  private Pose2d position;
  private CommandSwerveDrivetrain drivetrain;
  private Command driveToReef;
  private Command driveBack;
  private Command driveForward;
  private int phase = 0;
  private ElevatorPivot pivot = RobotContainer.elevatorPivot;
  private Claw claw = RobotContainer.claw;
  private Translation2d vector;
  private Translation2d reefCenter;
  private IntegerSubscriber poleHeightSubscriber;
  private Translation2d midpoint(Translation2d a, Translation2d b) {
    return new Translation2d(a.getX() + (b.getX() - a.getX())/2, a.getY() + (b.getY() - a.getY())/2);
  }
  private Pose2d findPose(Translation2d relative) {
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
      if (relative.getDistance(midpoints[i]) < relative.getDistance(closest)) {
        closest = midpoints[i];
        idx = i;
      }
    }
    return new Pose2d(closest, new Rotation2d(ROTATIONS[idx]));
  }

  /** Creates a new AutoScoreCoral. */
  public AutoScoreCoral(IntegerSubscriber PoleHeightSubscriber) {
    drivetrain = RobotContainer.drivetrain;
    position = findPose(drivetrain.getState().Pose.getTranslation());
    driveToReef = new DriveToPose(drivetrain, position);
    poleHeightSubscriber = PoleHeightSubscriber;
  }

  @Override
  public void initialize() {
    driveToReef.schedule();
  }

  @Override
  public void execute() {
    switch (phase) {
      case 0:
        if (driveToReef.isFinished()) {
          phase += 1;
        }
        break;
      case 1:
        //Drive back
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          reefCenter = REEF_CENTERS[0];
        } else {
          reefCenter = REEF_CENTERS[1];
        }
        vector = reefCenter.minus(position.getTranslation());
        vector = Translation2DUtils.normalize(vector);
        vector.times(BACKUP_DIST);
        driveBack = new DriveToPose(drivetrain, new Pose2d(position.getTranslation().plus(vector), position.getRotation()));
        driveBack.schedule();

        if (driveBack.isFinished()) {
          phase += 1;
        }
        break;
      case 2:
        //Move elvator up and turn
        long poleNum = poleHeightSubscriber.get();
        double height = SCORE_HEIGHTS[(int) poleNum];
        double rot = SCORE_ANGLES[(int) poleNum];
        pivot.goToPosition(() -> {return height;}, () -> {return rot;});

        if (pivot.atTargetAngle() && pivot.atTargetHeight()) {
          phase += 1;
        }
        break;
      case 3:
        //Drive forward
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          reefCenter = REEF_CENTERS[0];
        } else {
          reefCenter = REEF_CENTERS[1];
        }
        vector = reefCenter.minus(position.getTranslation());
        vector = Translation2DUtils.normalize(vector);
        vector.times(-BACKUP_DIST);
        driveForward = new DriveToPose(drivetrain, new Pose2d(position.getTranslation().plus(vector), position.getRotation()));
        driveForward.schedule();

        if (driveForward.isFinished()) {
          phase += 1;
        }
      case 4:
        //Score
        claw.outtake().until(() -> !ElevatorPivot.hasCoral()).andThen(claw.stop());
      case 5:
        //Drive back
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          reefCenter = REEF_CENTERS[0];
        } else {
          reefCenter = REEF_CENTERS[1];
        }
        vector = reefCenter.minus(position.getTranslation());
        vector = Translation2DUtils.normalize(vector);
        vector.times(BACKUP_DIST);
        driveBack = new DriveToPose(drivetrain, new Pose2d(position.getTranslation().plus(vector), position.getRotation()));
        driveBack.schedule();

        if (driveBack.isFinished()) {
          phase += 1;
        }
      case 6:
        //Reset
        pivot.stowArm();
        if (pivot.atTargetAngle() && pivot.atTargetHeight()) {
          phase += 1;
        }
      //Phase 7 means that the command is done
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return (phase >= 7);
  }
}
