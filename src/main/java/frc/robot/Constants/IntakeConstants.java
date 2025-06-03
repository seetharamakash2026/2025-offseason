package frc.robot.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class IntakeConstants{
    public static final int frontMotorID = 0;
    public static final int backMotorID = 1; 

    public static final String CAN_STRING = "Canivore 3045";
    public static final int backCancoderId = 2;

    

    public static final CurrentLimitsConfigs backMotorLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(null);
}
