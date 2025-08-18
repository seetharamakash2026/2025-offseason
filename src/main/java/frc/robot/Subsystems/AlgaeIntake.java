package frc.robot.Subsystems;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commons.GremlinLogger;
import frc.robot.commons.GremlinUtil;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.AlgaeIntakeConstants.*;

public class AlgaeIntake extends SubsystemBase{
    // two motors pivot and spinner thing basically the same as the other intake just different gear ratios n stuff maybe even copy paste the code and change the namess
    private TalonFX spinnerMotor = new TalonFX(spinMotorID, CAN_STRING); // define these
    private TalonFX pivotMotor = new TalonFX(pivotMotorID, CAN_STRING); 

    private CANcoder pivotCancoder = new CANcoder(backCancoderId, CAN_STRING);

    private double targetAngleDegrees;
    private double voltage; 

    public Trigger atTargetAngle = new Trigger(() -> atTargetAngle());

    public AlgaeIntake() {
        configDevices(); 

        if (Utils.isSimulation()) {
            configSimulation(); 
        }

        targetAngleDegrees = getAngleDegrees();
    }

    public void configDevices() {
        spinnerMotor.getConfigurator().apply(frontMotorConfig);
        pivotMotor.getConfigurator().apply(pivotMotorConfig);
        pivotCancoder.getConfigurator().apply(pivotCancoderConfig);

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
            pivotMotor.setControl(new TorqueCurrentFOC(current));
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
            .getStructArrayTopic("componentPoses", Pose3d.struct).publish();
        
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
            spinnerMotorSim = spinnerMotor.getSimState();
            pivotMotorSim = pivotMotor.getSimState();
            pivotCancoderSim = pivotCancoder.getSimState();

            spinnerMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
            pivotCancoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
            pivotMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

            flywheelSim.setInputVoltage(pivotMotorSim.getMotorVoltageMeasure().in(Volts));
            flywheelSim.update(0.02);

            singleJointedArmSim.setInputVoltage(pivotMotorSim.getMotorVoltageMeasure().in(Volts));
            singleJointedArmSim.update(0.02);


            pivotCancoderSim.setRawPosition(edu.wpi.first.math.util.Units.radiansToRotations((singleJointedArmSim.getAngleRads() * pivotSensorToMechanismRatio))); 
            pivotCancoderSim.setVelocity(edu.wpi.first.math.util.Units.radiansToRotations(singleJointedArmSim.getVelocityRadPerSec() * pivotSensorToMechanismRatio)); 
            pivotMotorSim.setRawRotorPosition(edu.wpi.first.math.util.Units.radiansToRotations(singleJointedArmSim.getAngleRads() * pivotTotalGearing));
            pivotMotorSim.setRotorVelocity(edu.wpi.first.math.util.Units.radiansToRotations(singleJointedArmSim.getVelocityRadPerSec() * pivotTotalGearing));
            
            updateMechanism2d();
        }

        public void updateMechanism2d() {
            double currentAngle = getAngleDegrees();

            if (GremlinLogger.DEBUG) { // ON BY DEFAULT
                pivotLigament.setAngle(180 - currentAngle); // I have a vague idea of what this is for

                componentPosesPublisher.set(new Pose3d[] {
                    new Pose3d(pivotOffsetX, 
                                pivotOffsetY, 
                                pivotOffsetZ, 
                                new Rotation3d(0, 
                                                -getangleRadians(), // same reason as the 180 - probably
                                                0))
                });
            }

            GremlinLogger.log("AlgaeIntake/Pivot", new Pose3d[] {
                new Pose3d(pivotOffsetX, 
                            pivotOffsetY,
                            pivotOffsetZ, 
                            new Rotation3d(0, 
                                            -getangleRadians(), // same reason as the 180 - probably
                                            0))
            });
        }

        private final SysIdRoutine m_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second),
                Volts.of(2),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    pivotMotor.setVoltage((volts.in(Volts)));
                }, 
                null, 
                this)
            );
                
        
        // possibly redundant
        public Command sysIdQuasistatic(SysIdRoutine.Direction d) {
            return m_SysIdRoutine.quasistatic(d);
        }
        public Command sysIdDynamic(SysIdRoutine.Direction d) {
            return m_SysIdRoutine.dynamic(d);
        }
}
