package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClawConstants.*;
import static frc.robot.Constants.ElevatorPivotConstants.canbus;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest.NativeSwerveRequest;


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
        motor.getConfigurator().apply(motorConfigs); // define

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

    public Command outtake() {
        return new SequentialCommandGroup(

        );
    }

    /* These should be gotten from elevatorpivot, since it has the sensors
    public boolean hasAlgae() {}
    public boolean hasCoral() {}
 */
}
