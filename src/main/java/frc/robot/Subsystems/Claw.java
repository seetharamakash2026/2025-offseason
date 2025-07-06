package frc.robot.Subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClawConstants.*;
import static frc.robot.Constants.ElevatorPivotConstants.canbus;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveRequest.NativeSwerveRequest;

// something
public class Claw extends SubsystemBase{ 
    @SuppressWarnings("unused")
    private TalonFX motor = new TalonFX(motorID, canbus);
    

    public Claw() {
        configDevices();

        if (Utils.isSimulation()) {
                configSimulation(); // define
            }
    }

    public void configDevices() {
        //motor.getConfigurator().apply(motorConfigs); // define

        motor.clearStickyFaults();

        BaseStatusSignal.setUpdateFrequencyForAll(200,
                motor.getPosition()
                );
    }

    public Command stop() {
        return this.run(() -> motor.stopMotor());
    }
    public Command intake() {
        return this.run(() -> motor.setVoltage(voltage)); // define
    }
    public Command outtake() {
        return this.run(() -> motor.setVoltage(-1 * voltage)); // define
    }

    public Command intakeCoral() {
        return new SequentialCommandGroup(
            intake()
            .until(() -> ElevatorPivot.hasCoral())
            .andThen(stop())
            );
    }

    public Command intakeAlgae() {
        return new SequentialCommandGroup(
            intake()
            .until(() -> ElevatorPivot.hasAlgae())
            .andThen(stop())
        );
    }


    public Command outtakeGeneric() {
        return new SequentialCommandGroup(
            outtake()
            .until(() -> !ElevatorPivot.hasAlgae() && !ElevatorPivot.hasCoral())
            .andThen(stop())
        );
    }

    /* These should be gotten from elevatorpivot, since it has the sensors
    public boolean hasAlgae() {}
    public boolean hasCoral() {}
 */

    // sim
    private final FlywheelSim flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 
            MOI, // define
            gearing), // define
        DCMotor.getKrakenX60(1));

    private TalonFXSimState motorSim;

    


    public void configSimulation() {
            motorSim = motor.getSimState();
            

            motorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public void simulationPeriodic() {
        motorSim = motor.getSimState();

        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        flywheelSim.setInputVoltage(motorSim.getMotorVoltageMeasure().in(Volts));
        flywheelSim.update(0.02);

        //updateMechanism2d()
    }
}
