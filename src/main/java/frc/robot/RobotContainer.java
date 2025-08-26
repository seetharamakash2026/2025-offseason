// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLogOptions;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.Commands.AutoScoreCoral;
import frc.robot.Commands.DriveToPose;
import frc.robot.RobotState.DriveState;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.ElevatorPivot;
import frc.robot.Subsystems.Claw;
import frc.robot.commons.GremlinLogger;
import frc.robot.commons.GremlinPS4Controller;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public static final RobotState M_ROBOT_STATE = RobotState.getRobotState();

  private final GremlinPS4Controller joystick = new GremlinPS4Controller(0);
  private final CommandGenericHID buttonBoard = new CommandGenericHID(1);
  public static final IntegerSubscriber poleHeightSubscriber = NetworkTableInstance.getDefault().getTable("Scoring Location").getIntegerTopic("Row").subscribe(0);
  
  // public final Intake intake = new Intake();

  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final static ElevatorPivot elevatorPivot = new ElevatorPivot();
  public final static Claw claw = new Claw();


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

    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
          drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
    );

    // joystick.L2().onTrue(
    //   new AutoScoreCoral(NetworkTableInstance.getDefault().getTable("Scoring Location").getIntegerTopic("Row").subscribe(0))
    // );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  // public void registerPathPlannerCommands(){
  //   NamedCommands.registerCommand("intake", 
  //     intake.goToAngleDegrees(IntakeConstants.minAngle)
  //       .andThen(intake.applyDownwardCurrentFront()) // could be upward who knows
  //       //.andThen(Commands.waitUntil(nextMechanism.hasCoral?))
  //       .andThen(intake.stow()).alongWith(intake.zeroCurrentFront())
  //     );
  // }
}
