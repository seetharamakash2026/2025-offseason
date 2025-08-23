package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Indexer extends SubsystemBase {
    private TalonFX motor = new TalonFX(motorID, CAN_STRING);

    private double voltage; 

    public Indexer() {
        configDevices(); // define
        if (Utils.isSimulation()) {
            configSimulation(); // define
        }
    }

    public void configDevices() {
        motor.getConfigurator().apply(motorConfig); // define
        
        motor.clearStickyFaults();

        BaseStatusSignal.setUpdateFrequencyForAll(200, 
            motor.getPosition());
    }

    public boolean spinning() {
        return motor.getRotorVelocity().getValueAsDouble() != 0; 
    }

    public Command applyDownwardCurrentSpinner() {
        return this.runOnce(() -> {
            double current = -10;
            motor.setControl(new TorqueCurrentFOC(current));
        });
    }
    
    public Command applyUpwardCurrentSpinner() {
        return this.runOnce(() -> {
            double current = 10;
            motor.setControl(new TorqueCurrentFOC(current));
        });
    }

    public Command zeroCurrentSpinner() {
        return this.runOnce(() -> {
            double current = 0;
            motor.setControl(new TorqueCurrentFOC(current));
        });
    }

    @Override
    public void periodic() {
        updateMechanism2d(); // define

        SmartDashboard.putBoolean("is spinning?", spinning());
    }

    // sim
    private final FlywheelSim flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 
        flywheelMOI, 
        flywheelGearing),
        DCMotor.getKrakenX60(1));

    private TalonFXSimState motorSim; 

    public void configSimulation() {
        motorSim = motor.getSimState();

        motorSim.Orientation = ChassisReference.CounterClockwise_Positive; // may need to fix this
    }
    
    @Override  
    public void simulationPeriodic() {
        motorSim = motor.getSimState();

        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        flywheelSim.setInputVoltage(motorSim.getMotorVoltageMeasure().in(Volts));
        flywheelSim.update(0.02);

        updateMechanism2d();
    }

    public void updateMechanism2d() {
        // could make logging stuff here but I don't feel like it
    }

    // could make sysid stuff here but I don't feel like it 
}
