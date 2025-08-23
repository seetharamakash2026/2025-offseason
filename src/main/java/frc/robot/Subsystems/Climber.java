package frc.robot.Subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commons.GremlinUtil;

import static frc.robot.Constants.AlgaeIntakeConstants.CAN_STRING;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.ElevatorPivotConstants.pivotSupplyCurrentLimitLowerLimit;

import java.io.ObjectInputFilter.Config;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
/*
 * TO DO:
 * Entire claw system to actually climb commands and everything past periodic 
 * figure out how the claw works in shop then it'll be simple
 * 
 */
public class Climber extends SubsystemBase {
    private TalonFX pivotMotor = new TalonFX(pivotID, CAN_STRING);
    private TalonFX wingsMotor = new TalonFX(wingsID, CAN_STRING);

    private CANcoder pivotCancoder = new CANcoder(pivotCancoderID, CAN_STRING);

    private double targetAngleDegrees;
    private double voltage; 

    public Trigger atTargetAngle = new Trigger(() -> atTargetAngle()); // define
    
    public Climber() {
        configDevices(); // define

        if (Utils.isSimulation()) {
            configSimulation(); // define
        }

        targetAngleDegrees = getAngleDegrees(); // define
    }

    public void configDevides() {
        pivotMotor.getConfigurator().apply(pivotMotorConfig); // define
        wingsMotor.getConfigurator().apply(wingMotorConfig); // define
        pivotCancoder.getConfigurator().apply(pivotCancoderConfig); // define

        pivotMotor.clearStickyFaults();
        wingsMotor.clearStickyFaults();

        BaseStatusSignal.setUpdateFrequencyForAll(200,
            pivotMotor.getPosition(),
            wingsMotor.getPosition(),
            pivotCancoder.getPosition());
    }

    public double getangleRotations() {
        return pivotCancoder.getPosition().getValueAsDouble();
    }

    public double getAngleDegrees() {
        return Units.rotationsToDegrees(pivotCancoder.getPosition().getValueAsDouble());
    }

    public double getAngleRadians() {
        return Units.rotationsToRadians(pivotCancoder.getPosition().getValueAsDouble());
    }

    public boolean atTargetAngle() {
        return Math.abs(getAngleDegrees() - targetAngleDegrees) < angleToleranceDegrees; 
    }

    private void setAngleTargetDegrees(double targetAngleDegrees) {
        this.targetAngleDegrees = GremlinUtil.clampWithLogs(maxAngle, minAngle, targetAngleDegrees);

        double targetAngleRotations = Units.degreesToRadians(targetAngleDegrees);

        MotionMagicVoltage request = new MotionMagicVoltage(targetAngleRotations)
            .withEnableFOC(true)
            .withSlot(0)
            .withUpdateFreqHz(1000);
        
        pivotMotor.setControl(request);
    }



    // commands
    public Command goToAngleDegrees(DoubleSupplier angle) {
        return this.run(() -> {
            setAngleTargetDegrees(angle.getAsDouble());
        }).until(atTargetAngle);
    }

    public Command goToAngleDegrees(double angle) {
        return this.run(() -> {
            setAngleTargetDegrees(angle);
        }).until(atTargetAngle);
    }

    public Command max() {
        return goToAngleDegrees(maxAngle);
    }
    public Command min() {
        return goToAngleDegrees(minAngle);
    }

    public Command increaseVoltage() {
        return this.runOnce(() -> {
            voltage += 0.1; 
            pivotMotor.setVoltage(voltage);
            SmartDashboard.putNumber("voltage", voltage);
        });
    }
    public Command decreaseVoltage() {
        return this.runOnce(() -> {
            voltage -= 0.1; 
            pivotMotor.setVoltage(voltage);
            SmartDashboard.putNumber("voltage", voltage);
        });
    }
    public Command zeroVoltage() {
        return this.runOnce(() -> {
            voltage = 0; 
            pivotMotor.setVoltage(voltage);
            SmartDashboard.putNumber("voltage", voltage);
        }); 
    }
    public Command applyDownwardCurrent() {
        return this.runOnce(() ->  {
            double current = -10;
            pivotMotor.setControl(new TorqueCurrentFOC(current));
        });
    }
    public Command applyUpwardCurrent() {
        return this.runOnce(() -> {
            double current = -10;
            pivotMotor.setControl(new TorqueCurrentFOC(current));
        });
    }
    public Command zeroCurrent() {
        return this.runOnce(() -> {
            double current = 0; 
            pivotMotor.setControl(new TorqueCurrentFOC(current));
        });
    }

    @Override
    public void periodic() {
        updateMechanism2d(); // define 
        
        SmartDashboard.putNumber("pivot angle", getAngleDegrees());
    }
}
