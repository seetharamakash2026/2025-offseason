// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.PathfindCommandConstants;
//ALL SYNTAX ERRORS HERE WILL BE RESOLVED ONCE WE ACTUALLY ADD PHOENIXTUNER FILES
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.constants.DriveConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathfindCommand extends Command {
  /** Creates a new PathfindCommand. */
  private Command m_PathfindingCommand = AutoBuilder.pathfindToPose(
    Pose2d.kZero,
    DriveConstants.pathFollowingConstraints,
    PathfindCommandConstants.kMaxVelError // Goal end velocity in meters/sec
  );

  private static Pose2d m_targetPose = new Pose2d();

  AutoBuilder m_AutoBuilder = new AutoBuilder();
  public PathfindCommand(CommandSwerveDrivetrain drivetrainRef, Pose2d targetPose) {
    m_targetPose = targetPose;
    RobotConfig config = new RobotConfig(0, 0, null, Translation2d.kZero);
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace(); //if this doesn't load then the entire path planner system is screwed
    }
    
    AutoBuilder.configure(
      () -> {return m_DrivetrainRef.getState().Pose;},
      (Pose2d pose) -> {m_DrivetrainRef.resetPose(pose);},
      () -> {return m_DrivetrainRef.getState().Speeds;},
      (ChassisSpeeds speeds, DriveFeedforwards feedForward) -> {drivetrainRef.driveRobotRelative(speeds);},
      pathFollowingController,
      config,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      m_DrivetrainRef);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PathfindingCommand = AutoBuilder.pathfindToPose(
      m_targetPose,
      DriveConstants.pathFollowingConstraints,
      PathfindCommandConstants.kMaxVelError // Goal end velocity in meters/sec
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
