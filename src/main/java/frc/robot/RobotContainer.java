// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotState.DriveState;
import frc.robot.Subsystems.Intake;
import frc.robot.commons.GremlinLogger;
import frc.robot.commons.GremlinPS4Controller;

public class RobotContainer {
  public static final RobotState M_ROBOT_STATE = RobotState.getRobotState();

  private final GremlinPS4Controller joystick = new GremlinPS4Controller(0);
  private final CommandGenericHID buttonBoard = new CommandGenericHID(1);
  
  public final Intake intake = new Intake();


  public final Trigger intakeState = new Trigger(() -> M_ROBOT_STATE.getDriveState() == DriveState.INTAKE);
  public final Trigger teleopState = new Trigger(() -> M_ROBOT_STATE.getDriveState() == DriveState.TELEOP);


  public RobotContainer() {
    GremlinLogger.setOptions(new DogLogOptions()
      .withNtPublish(false)
      .withCaptureNt(true)
      .withCaptureConsole(true)
      .withLogExtras(false));
    
    // auto stuff goes here when made

    configureBindings();
  }

  private void configureBindings() {
    
    joystick.R1().onTrue(
      new ConditionalCommand(
        Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.INTAKE)),
        Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.TELEOP)), 
        intakeState.negate()));
    

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void registerPathPlannerCommands(){
    NamedCommands.registerCommand("intake", 
      intake.goToAngleDegrees(IntakeConstants.minAngle)
        .andThen(intake.applyDownwardCurrentFront()) // could be upward who knows
        //.andThen(Commands.waitUntil(nextMechanism.hasCoral?))
        .andThen(intake.stow()).alongWith(intake.zeroCurrentFront())
      );
  }
}
