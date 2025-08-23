package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

public class ClimberConstants {
    public static final int pivotID = 20;
    public static final int wingsID = 16; // these might need to be switched around

    public static final String canbus = "Canivore 3045";
    public static final int pivotCancoderID = 20;

    public static final String climberTable = "climber";

    public static final double canvasHeight = 2;
    public static final double canvasWidth = 6;

    public static final double timesyncFrequency = 200; 

    public static final double angleToleranceDegrees = 5; 
    public static final double maxAngle = 90; 
    public static final double minAngle = 0; 

    public static final double pivotOffsetX = Units.inchesToMeters(0);
    public static final double pivotOffsetY = Units.inchesToMeters(0);
    public static final double pivotOffsetZ = Units.inchesToMeters(0);
    
    public static final double pivotRotorToSensorRatio = 1; // no idea how this is calculated
    public static final double pivotSensorToMechanismRatio = 0; 
    public static final double pivotTotalGearing = pivotSensorToMechanismRatio * pivotRotorToSensorRatio;
    public static final double pivotMOI = 0.0000000001; // probably make this a real value? 
    public static final double intakeLength = Units.inchesToMeters(0);

    
}
