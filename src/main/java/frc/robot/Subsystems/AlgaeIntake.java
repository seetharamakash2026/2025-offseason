package frc.robot.Subsystems;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commons.GremlinUtil;

public class AlgaeIntake extends SubsystemBase{
    // two motors pivot and spinner thing basically the same as the other intake just different gear ratios n stuff maybe even copy paste the code and change the namess
    private TalonFX spinnerMotor = new TalonFX(spinMotorID, CAN_STRING); // define these
    private TalonFX pivotMotor = new TalonFX(pivotMotorID, CAN_STRING); 

    private CANcoder pivotCancoder = new CANcoder(pivotCancoderID, CAN_STRING);

    private double targetAngleDegrees;
    private double voltage; 

    public Trigger atTargetAngle = new Trigger(() -> atTargetAngle());

    public Intake() {
        configDevices(); // define

        if (Utils.isSimulation()) {
            configSimulation(); // define
        }

        targetAngleDegrees = getAngleDegrees();
    }

    public void configDevices() {
        spinnerMotor.getConfigurator().apply(spinnerMotorConfig);
        pivotMotor.getConfigurator().apply(pivotMotorConfig);
        pivotCancoder.getConfigurator().apply(pivotCandoderConfig);

        spinnerMotor.clearStickyFaults();
        pivotMotor.clearStickyFaults();

        BaseStatusSignal.setUpdateFrequencyForAll(200,
            spinnerMotor.getPosition(),
            pivotMotor.getPosition(),
            pivotCancoder.getPosition());
    }

    public boolean spinning() {
        return spinnerMotor.getRotorVelocity().getValueAsDouble() != 0; 
    }

    public double getangleRotations() {
        return Units.degreesToRadians(pivotCancoder.getPosition().getValueAsDouble());
    }

    public double getAngleDegrees() {
        return Units.rotationsToDegrees(pivotCancoder.getPosition().getValueAsDouble());
    }

    public double getangleRadians() {
        return Units.rotationsToRadians(pivotCancoder.getPosition().getValueAsDouble());
    }

    public boolean atTargetAngle() {
        return Math.abs(getAngleDegrees() - targetAngleDegrees) < angleToleranceDegrees; // DEFINE
    }

    private void setAngleTargetDegrees(double targetAngleDegrees) {
        this.targetAngleDegrees = GremlinUtil.clampWithLogs(maxAngle, minAngle, targetAngleDegrees);
    
        double targetAngleRotations = edu.wpi.first.math.util.Units.degreesToRadians(targetAngleDegrees);

        MotionMagicVoltage request = new MotionMagicVoltage(targetAngleRotations)
            .withEnableFOC(true).withSlot(0).withUpdateFreqHz(1000);
        
            pivotMotor.setControl(request);
    }

    // Commands

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

    public Command stow() {
        return goToAngleDegrees(maxAngle);
    }

    public Command lower() {
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
        this.runOnce(() ->  {
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
            pivotMotor.setControl(new TorquecurrentFOC(current));
        });
    }
    
    // spins wheels motor (front)
    public Command applyDownwardCurrentSpinner() {
        return this.runOnce(() -> {
            double current = -10;
            spinnerMotor.setControl(new TorqueCurrentFOC(current));
        });
    }
    
    public Command applyUpwardCurrentSpinner() {
        return this.runOnce(() -> {
            double current = 10;
            spinnerMotor.setControl(new TorqueCurrentFOC(current));
        });
    }

    public Command zeroCurrentSpinner() {
        return this.runOnce(() -> {
            double current = 0;
            spinnerMotor.setControl(new TorqueCurrentFOC(current));
        });
    }

    @Override
    public void periodic() {
        updateMechanism2d(); // define I think who knows at this point I want to sleeeeeeeeeeeep
        
        SmartDashboard.putNumber("pivot angle", getAngleDegrees());
        SmartDashboard.putBoolean("is spinning?", spinning()); 
    }

    // sim
    private final FlywheelSim flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 
            flywheelMOI, // define
            flywheelGearing),
        DCMotor.getKrakenX60(1)); // define

    private final SingleJointedArmSim singleJointedArmSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1), 
        pivotTotalGearing, 
        pivotMOI, 
        intakeLength, 
        edu.wpi.first.math.util.Units.degreesToRadians(minAngle),
        edu.wpi.first.math.util.Units.degreesToRadians(maxAngle),
        true,
        edu.wpi.first.math.util.Units.degreesToRadians(minAngle));
    
        private TalonFXSimState spinnerMotorSim;
        private TalonFXSimState pivotMotorSim; 
        private CANcoderSimState pivotCancoderSim;

        private Mechanism2d pivotMech = new Mechanism2d(canvasWidth, canvasHeight);
        private StructArrayPublisher<Pose3d> componentPosesPublisher = NetworkTableInstance.getDefault()
            .getTable(algaeIntakeTable)
        
        private MechanismRoot2d pivotRoot = pivotMech.getRoot(("pivotRoot"), 2, 3); // as with the intake I don't know what the 2 and 3 are
        private MechanismLigament2d pivotLigament = pivotRoot.append(
            new MechanismLigament2d("algaeIntakeLigament", intakeLength, minAngle)
        );

        public void configSimulation() {
            spinnerMotorSim = spinnerMotor.getSimState();
            pivotMotorSim = pivotMotor.getSimState();
            pivotCancoderSim = pivotCancoder.getSimState();

            spinnerMotorSim.Orientation = ChassisReference.CounterClockwise_Positive; // maybe fix
            pivotMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;

            singleJointedArmSim.setState(0,0);
        }

        @Override
        public void simulationPeriodic() {
            
        }
}
