// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CommandSwerveDrivetrain;

import static frc.robot.Constants.DriveConstants.*;

public class DriveToPose extends Command {
  private final ProfiledPIDController translationXController = new ProfiledPIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD, TRANSLATION_CONSTRAINTS);
  private final ProfiledPIDController translationYController = new ProfiledPIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD, TRANSLATION_CONSTRAINTS);
  private final ProfiledPIDController rotationController = new ProfiledPIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD, ROTATION_CONSTRAINTS);
  private CommandSwerveDrivetrain drivetrain;
  private Pose2d targetPose;
  /** Creates a new DriveToPose. */
  public DriveToPose(CommandSwerveDrivetrain Drivetrain, Pose2d TargetPose) {
    drivetrain = Drivetrain;
    targetPose = TargetPose;
  }

  private Pose2d getCurrPose() {
    return drivetrain.getState().Pose;
  }

  private Translation2d getDistFromTarget() {
    Pose2d currentPose = getCurrPose();
    return targetPose.getTranslation().minus(currentPose.getTranslation());
  }

  private double getRotationalDistFromTarget() {
    Pose2d currentPose = getCurrPose();
    return targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Translation2d distFromTarget = getDistFromTarget();
    double distFromRotationalTarget = getRotationalDistFromTarget();
    
    translationXController.reset(distFromTarget.getX());
    translationYController.reset(distFromTarget.getY());
    rotationController.reset(distFromRotationalTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = getCurrPose();

    //Logging
    SmartDashboard.putNumberArray("CurrentPose", new double[]{currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees()});
    SmartDashboard.putNumberArray("TargetPose", new double[]{targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()});

    Translation2d distFromTarget = getDistFromTarget();
    double distFromRotationalTarget = getRotationalDistFromTarget();
    
    // Drive the robot, the PID controllers don't mean shit without actually applying the speeds to the drivetrain

    drivetrain.driveRelativeSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
      translationXController.calculate(distFromTarget.getX()),
      translationYController.calculate(distFromTarget.getY()),
      rotationController.calculate(distFromRotationalTarget),
      currentPose.getRotation()
    ));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Cancel robot movement after command is done
    drivetrain.driveRelativeSpeeds(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return translationXController.atSetpoint() && translationYController.atSetpoint() && rotationController.atSetpoint();
  }
}
