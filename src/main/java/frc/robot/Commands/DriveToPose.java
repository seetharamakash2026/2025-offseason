// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;

import static frc.robot.Constants.DriveConstants.*;

public class DriveToPose extends Command {
  private final ProfiledPIDController translationXController = new ProfiledPIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD, TRANSLATION_CONSTRAINTS);
  private final ProfiledPIDController translationYController = new ProfiledPIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD, TRANSLATION_CONSTRAINTS);
  private final ProfiledPIDController rotationController = new ProfiledPIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD, ROTATION_CONSTRAINTS);
  private CommandSwerveDrivetrain drivetrain;
  private Pose2d targetPose;
  /** Creates a new DriveToPose command. */
  public DriveToPose(CommandSwerveDrivetrain Drivetrain, Pose2d TargetPose) {
    drivetrain = Drivetrain;
    targetPose = TargetPose;
  }

  private Pose2d getCurrPose() {
    return drivetrain.getState().Pose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    translationXController.setGoal(targetPose.getX());
    translationXController.setTolerance(0.1, 0.1);
    translationYController.setGoal(targetPose.getY());
    translationYController.setTolerance(0.1, 0.1);
    rotationController.reset(getCurrPose().getRotation().getRadians());
    rotationController.setGoal(targetPose.getRotation().getRadians());
    rotationController.setTolerance(0.1, 0.1);
    rotationController.disableContinuousInput();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = getCurrPose();

    //Logging
    SmartDashboard.putNumberArray("CurrentPose", new double[]{currentPose.getX(), currentPose.getY(), currentPose.getRotation().getRadians()});
    SmartDashboard.putNumberArray("TargetPose", new double[]{targetPose.getX(), targetPose.getY(), targetPose.getRotation().getRadians()});
    
    // Drive the robot, the PID controllers don't mean shit without actually applying the speeds to the drivetrain

    double xSpeed = translationXController.calculate(currentPose.getX());
    double ySpeed = translationYController.calculate(currentPose.getY());
    double rotationalSpeed = rotationController.calculate(currentPose.getRotation().getRadians());

    SmartDashboard.putNumberArray("Attempted solve",
      new Double[]{xSpeed, ySpeed, rotationalSpeed}
    );

    drivetrain.setControl(DriveConstants.APPLY_FIELD_SPEEDS.withSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rotationalSpeed)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Cancel robot movement after command is done
    drivetrain.setControl(DriveConstants.APPLY_FIELD_SPEEDS.withSpeeds(new ChassisSpeeds(0, 0, 0)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return translationXController.atSetpoint() && translationYController.atSetpoint() && rotationController.atSetpoint();
  }
}
