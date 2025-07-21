// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.DriveToPose;
import frc.robot.Subsystems.CommandSwerveDrivetrain;

import static frc.robot.Constants.AutoScoreConstants.*;

import org.dyn4j.geometry.Rotatable;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScoreCoral extends Command {
  private Pose2d position;
  private CommandSwerveDrivetrain drivetrain;
  private Command driveToPose;
  private int phase = 0;
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
  public AutoScoreCoral(CommandSwerveDrivetrain Drivetrain) {
    drivetrain = Drivetrain;
    position = findPose(drivetrain.getState().Pose.getTranslation());
    driveToPose = new DriveToPose(drivetrain, position);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveToPose.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (phase) {
      case 0:
        if (driveToPose.isFinished()) {
          phase = 1;
        }
        break;
      case 1:
        //Drive back
        break;
      case 2:
        //Score
        break;
      //Phase 3 means that the command is done
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (phase == 3);
  }
}
