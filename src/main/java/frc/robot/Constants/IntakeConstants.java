package frc.robot.Constants;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.util.Units;

public class IntakeConstants{
    public static final int frontMotorID = 16;
    public static final int backMotorID = 17; 

    public static final String CAN_STRING = "Canivore 3045";
    public static final int backCancoderId = 2;

    public static final String intakeTable = "intake"; 

    public static final double canvasHeight = 2; 
    public static final double canvasWidth = 6; 

    public static final double timesyncFrequency = 200; // Hz

    public static final double pivotRotorToSensorRatio = 1; // no idea how this is calculated
    public static final double pivotSensorToMechanismRatio = 1;
    public static final double pivotTotalGearing = pivotSensorToMechanismRatio * pivotRotorToSensorRatio;
    public static final double pivotMOI = 0.0000000001; // probably make this a real value? 
    public static final double intakeLength = Units.inchesToMeters(19.712057);

    public static final double angleToleranceDegrees = 5; 

    public static final double maxAngle = 90; 
    public static final double minAngle = 0; 

    public static final double pivotOffsetX = Units.inchesToMeters(7.441);
    public static final double pivotOffsetY = Units.inchesToMeters(5.940080);
    public static final double pivotOffsetZ = Units.inchesToMeters(-1.189711);

    // for front motor, which spins the wheels
    public static final double frontStatorCurrentLimit = 60; // Amps
    public static final double frontSupplyCurrentLimit = 80; // Amps
    public static final double frontSupplyCurrentLimitLowerLimit = 60; // Amps
    public static final double frontSupplyCurrentLimitLowerLimitTime = 1; // Seconds
    public static final boolean frontStatorCurrentLimitEnable = true;
    public static final boolean frontSupplyCurrentLimitEnable = true; 

    public static final InvertedValue frontInvert = InvertedValue.CounterClockwise_Positive; 

    public static final double flywheelMOI = 0.00000000001; // probably make this a real value?
    public static final double flywheelGearing = 44 / 12;

    public static final CurrentLimitsConfigs frontCurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(frontStatorCurrentLimit)
            .withSupplyCurrentLimit(frontSupplyCurrentLimit)
            .withStatorCurrentLimitEnable(frontStatorCurrentLimitEnable)
            .withSupplyCurrentLimitEnable(frontSupplyCurrentLimitEnable)
            .withSupplyCurrentLowerTime(frontSupplyCurrentLimitLowerLimitTime)
            .withSupplyCurrentLowerLimit(frontSupplyCurrentLimitLowerLimit);
    public static final MotorOutputConfigs frontMotorOutputConfigs = new MotorOutputConfigs()
            .withControlTimesyncFreqHz(timesyncFrequency)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(frontInvert);
    public static final TalonFXConfiguration frontMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(frontCurrentLimits)
            .withMotorOutput(frontMotorOutputConfigs);


    
    
    // for back motor, which acts as the pivot
    public static final double pivotStatorCurrentLimit = 60; // Amps
    public static final double pivotSupplyCurrentLimit = 80; // Amps
    public static final double pivotSupplyCurrentLimitLowerLimit = 60; // Amps
    public static final double pivotSupplyCurrentLimitLowerLimitTime = 1; // second
    public static final boolean pivotStatorCurrentLimitEnable = true;
    public static final boolean pivotSupplyCurrentLimitEnable = true;

    public static final double pivotMaxAcceleration = 2; // rot per sec^2 possibly edit these
    public static final double pivotMaxVelocity = 2; // rot per sec

    public static final InvertedValue pivotInvert = InvertedValue.CounterClockwise_Positive;

    public static final double pivotKP = !Utils.isSimulation() ? 0 : 0;
    public static final double pivotKI = !Utils.isSimulation() ? 0 : 0;
    public static final double pivotKD = !Utils.isSimulation() ? 0 : 0;
    public static final double pivotKG = !Utils.isSimulation() ? 0 : 0;
    public static final double pivotKS = !Utils.isSimulation() ? 0 : 0;
    public static final double pivotKA = !Utils.isSimulation() ? 0 : 0;
    public static final double pivotKV = !Utils.isSimulation() ? 0 : 0;

    public static final double magnetOffset = !Utils.isSimulation() ? 0 : 0; // real value should be positive
    public static final SensorDirectionValue pivotEncoderSensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    public static final CurrentLimitsConfigs pivotCurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(pivotStatorCurrentLimit)
            .withSupplyCurrentLimit(pivotSupplyCurrentLimit)
            .withStatorCurrentLimitEnable(pivotStatorCurrentLimitEnable)
            .withSupplyCurrentLimitEnable(pivotSupplyCurrentLimitEnable)
            .withSupplyCurrentLowerTime(pivotSupplyCurrentLimitLowerLimitTime)
            .withSupplyCurrentLowerLimit(pivotSupplyCurrentLimitLowerLimit);

    public static final FeedbackConfigs pivotFeedbackConfigs = new FeedbackConfigs()
            .withFusedCANcoder(new CoreCANcoder(backCancoderId, CAN_STRING))
            .withRotorToSensorRatio(pivotRotorToSensorRatio)
            .withSensorToMechanismRatio(pivotSensorToMechanismRatio);

    public static final MotionMagicConfigs pivotMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(pivotMaxAcceleration)
            .withMotionMagicCruiseVelocity(pivotMaxVelocity); // Consider adding jerk or making it expo

    public static final MotorOutputConfigs pivotMotorOutputConfigs = new MotorOutputConfigs()
            .withControlTimesyncFreqHz(timesyncFrequency)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(pivotInvert);

    public static final Slot0Configs pivotSlot0Configs = new Slot0Configs()
            .withKA(pivotKA)
            .withKD(pivotKD)
            .withKG(pivotKG)
            .withKI(pivotKI)
            .withKP(pivotKP)
            .withKS(pivotKS)
            .withKV(pivotKV)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withGravityType(GravityTypeValue.Arm_Cosine);

    public static final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(pivotCurrentLimits)
            .withFeedback(pivotFeedbackConfigs)
            .withMotionMagic(pivotMotionMagicConfigs)
            .withMotorOutput(pivotMotorOutputConfigs)
            .withSlot0(pivotSlot0Configs);

    public static final CANcoderConfiguration pivotCancoderConfig = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                    .withSensorDirection(pivotEncoderSensorDirection)
                    .withAbsoluteSensorDiscontinuityPoint(0.5) // [-0.5,0.5]
                    .withMagnetOffset(magnetOffset));
}
