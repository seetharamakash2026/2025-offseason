package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoderSimCollection;
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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static frc.robot.Constants.IntakeConstants.*;
import frc.robot.commons.GremlinLogger;
import frc.robot.commons.GremlinUtil;
import frc.robot.commons.GremlinLogger;

import swervelib.encoders.CanAndMagSwerve;

import static edu.wpi.first.units.Units.*;

import java.lang.Thread.State;
import java.util.function.DoubleSupplier;

import org.opencv.dnn.Model;

public class Intake extends SubsystemBase{
        @SuppressWarnings("unused")
        private TalonFX frontMotor = new TalonFX(frontMotorID, CAN_STRING);
        private TalonFX backMotor = new TalonFX(backMotorID, CAN_STRING);
        private CANcoder backCancoder = new CANcoder(backCancoderId, CAN_STRING);

        private double targetAngleDegrees; 
        private double voltage;

        public Trigger atTargetAngle = new Trigger(() -> atTargetAngle()); 

        public Intake() {
            configDevices(); 

            if (Utils.isSimulation()) {
                configSimulation(); 
            }

            targetAngleDegrees = getAngleDegrees(); 
        }

        // configures everything and zeroes
        public void configDevices() {
            frontMotor.getConfigurator().apply(frontMotorConfig); 
            backMotor.getConfigurator().apply(pivotMotorConfig); 
            backCancoder.getConfigurator().apply(pivotCancoderConfig);

            backMotor.clearStickyFaults();
            frontMotor.clearStickyFaults();

            frontMotor.setPosition(0); // redundant ?
            backMotor.setPosition(0);

            BaseStatusSignal.setUpdateFrequencyForAll(200,
                frontMotor.getPosition(),
                backMotor.getPosition(),
                backCancoder.getPosition()
                );
        }
        
        public boolean spinning() {
            return frontMotor.getRotorVelocity().getValueAsDouble() != 0;
        }

        public double getAngleRotations() {
            return edu.wpi.first.math.util.Units.degreesToRotations(backCancoder.getPosition().getValueAsDouble());
            //return backMotor.getPosition().getValueAsDouble();
        } 

        public double getAngleDegrees() {
            return edu.wpi.first.math.util.Units.rotationsToDegrees(backCancoder.getPosition().getValueAsDouble());
        }

        public double getAngleRadians() {
            return edu.wpi.first.math.util.Units.rotationsToRadians(backCancoder.getPosition().getValueAsDouble());
        }

        public boolean atTargetAngle() {
            return Math.abs(getAngleDegrees() - targetAngleDegrees) < angleToleranceDegrees; 
        }
        
        private void setAngleTargetDegrees(double targetAngleDegrees) {
            this.targetAngleDegrees = GremlinUtil.clampWithLogs(maxAngle, minAngle, targetAngleDegrees); 
            
            double targetAngleRotations = edu.wpi.first.math.util.Units.degreesToRotations(targetAngleDegrees);

            MotionMagicVoltage request = new MotionMagicVoltage(targetAngleRotations)
                .withEnableFOC(true).withSlot(0).withUpdateFreqHz(1000); 

            backMotor.setControl(request);
        }

        /*
         * COMMANDS
         */

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
            return goToAngleDegrees(minAngle); 
        }

        public Command activateForCoral() {
            return goToAngleDegrees(maxAngle); 
        }
        /*
         * Voltage and Current
         */
        

        // for pivot
        public Command increaseVoltage() {
            return this.runOnce(() -> {
                voltage += 0.1;
                backMotor.setVoltage(voltage);
                SmartDashboard.putNumber("voltage", voltage);
            });
        }
        public Command decreaseVoltage() {
            return this.runOnce(() -> {
                voltage -= 0.1;
                backMotor.setVoltage(voltage);
                SmartDashboard.putNumber("voltage", voltage);
            });
        }
        
        public Command zeroVoltage() {
            return this.runOnce(() -> {
                voltage = 0;
                backMotor.setVoltage(voltage);
                SmartDashboard.putNumber("voltage", voltage);
            });
        }
        
        public Command applyDownwardCurrent() {
            return this.runOnce(() -> {
                double current = -10;
                backMotor.setControl(new TorqueCurrentFOC(current));
            });
        }
        
        public Command applyUpwardCurrent () {
            return this.runOnce(() -> {
                double current = 10;
                backMotor.setControl(new TorqueCurrentFOC(current));
            });
        }

        public Command zeroCurrent () {
            return this.runOnce(() -> {
                double current = 0;
                backMotor.setControl(new TorqueCurrentFOC(current));
            });
        }
        
        // spins wheels motor (front)
        public Command applyDownwardCurrentFront() {
            return this.runOnce(() -> {
                double current = -10;
                frontMotor.setControl(new TorqueCurrentFOC(current));
            });
        }
        
        public Command applyUpwardCurrentFront() {
            return this.runOnce(() -> {
                double current = 10;
                frontMotor.setControl(new TorqueCurrentFOC(current));
            });
        }

        public Command zeroCurrentFront() {
            return this.runOnce(() -> {
                double current = 0;
                frontMotor.setControl(new TorqueCurrentFOC(current));
            });
        }

        @Override
        public void periodic() {
            updateMechanism2d(); 

            SmartDashboard.putNumber("pivot angle", getAngleDegrees());
            SmartDashboard.putBoolean("rarely is the question asked, is our children spinning?", spinning());
        }

        // sim
        private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), flywheelMOI, flywheelGearing),
            DCMotor.getKrakenX60(1)
        );
        private final SingleJointedArmSim singleJointedArmSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            pivotTotalGearing, 
            pivotMOI, 
            intakeLength,  
            edu.wpi.first.math.util.Units.degreesToRadians(minAngle),
            edu.wpi.first.math.util.Units.degreesToRadians(maxAngle), 
            true, 
            edu.wpi.first.math.util.Units.degreesToRadians(minAngle)
            );
        
        private TalonFXSimState frontMotorSim;
        private TalonFXSimState backMotorSim;
        private CANcoderSimState backCancoderSim;

        private Mechanism2d pivotMech = new Mechanism2d(canvasWidth, canvasHeight); 
        private StructArrayPublisher<Pose3d> componentPosesPublisher = NetworkTableInstance.getDefault()
            .getTable(intakeTable)
            .getStructArrayTopic("componentPoses", Pose3d.struct).publish();
        
        private MechanismRoot2d pivotRoot = pivotMech.getRoot(("pivotRoot"), 2, 3); // I don't know what the 2 and 3 are for I think they might just be replaced
        private MechanismLigament2d pivotLigament = pivotRoot.append(
            new MechanismLigament2d("pivotLigament", intakeLength, minAngle) 
        ); 

        public void configSimulation() {
            frontMotorSim = frontMotor.getSimState();
            backMotorSim = backMotor.getSimState();
            backCancoderSim = backCancoder.getSimState();

            frontMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
            backMotorSim.Orientation = ChassisReference.CounterClockwise_Positive; // be ready to fix these

            singleJointedArmSim.setState(0, 0);
        }

        @Override
        public void simulationPeriodic() {
            frontMotorSim = frontMotor.getSimState();
            backMotorSim = backMotor.getSimState();
            backCancoderSim = backCancoder.getSimState();

            frontMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
            backCancoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
            backMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

            flywheelSim.setInputVoltage(frontMotorSim.getMotorVoltageMeasure().in(Volts));
            flywheelSim.update(0.02);

            singleJointedArmSim.setInputVoltage(backMotorSim.getMotorVoltageMeasure().in(Volts));
            singleJointedArmSim.update(0.02);


            backCancoderSim.setRawPosition(edu.wpi.first.math.util.Units.radiansToRotations((singleJointedArmSim.getAngleRads() * pivotSensorToMechanismRatio))); 
            backCancoderSim.setVelocity(edu.wpi.first.math.util.Units.radiansToRotations(singleJointedArmSim.getVelocityRadPerSec() * pivotSensorToMechanismRatio)); 
            backMotorSim.setRawRotorPosition(edu.wpi.first.math.util.Units.radiansToRotations(singleJointedArmSim.getAngleRads() * pivotTotalGearing));
            backMotorSim.setRotorVelocity(edu.wpi.first.math.util.Units.radiansToRotations(singleJointedArmSim.getVelocityRadPerSec() * pivotTotalGearing));
            
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
                                                -getAngleRadians(), // same reason as the 180 - probably
                                                0))
                });
            }

            GremlinLogger.log("Intake/Pivot", new Pose3d[] {
                new Pose3d(pivotOffsetX, 
                            pivotOffsetY,
                            pivotOffsetZ, 
                            new Rotation3d(0, 
                                            -getAngleRadians(), // same reason as the 180 - probably
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
                    backMotor.setVoltage((volts.in(Volts)));
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
