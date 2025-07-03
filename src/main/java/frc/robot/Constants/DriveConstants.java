// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/** Add your docs here. */
public class DriveConstants {
    public static final double TRANSLATION_KP = 1;
    public static final double TRANSLATION_KI = 0;
    public static final double TRANSLATION_KD = 0;

    public static final double ROTATION_KP = 1;
    public static final double ROTATION_KI = 0;
    public static final double ROTATION_KD = 0;

    public static final double MAX_VELOCITY = 5;
    public static final double MAX_ACCELERATION = 2;

    public static final double MAX_ROTATIONAL_VELOCITY = 1;
    public static final double MAX_ROTATIONAL_ACCELERATION = 1;

    public static final Constraints TRANSLATION_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
    public static final Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ROTATIONAL_VELOCITY, MAX_ROTATIONAL_ACCELERATION);

    public static final SwerveRequest.ApplyFieldSpeeds APPLY_FIELD_SPEEDS = new ApplyFieldSpeeds()
            .withDesaturateWheelSpeeds(true)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity);
}