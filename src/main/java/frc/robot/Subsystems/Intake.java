package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.IntakeConstants;
import frc.robot.commons.GremlinLogger;
import frc.robot.commons.GremlinLogger;

import swervelib.encoders.CanAndMagSwerve;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import org.opencv.dnn.Model;

public class Intake extends SubsystemBase{
        @SuppressWarnings("unused")
        private TalonFX frontMotor = new TalonFX(IntakeConstants.frontMotorID, IntakeConstants.CAN_STRING);
        private TalonFX backMotor = new TalonFX(IntakeConstants.backMotorID, IntakeConstants.CAN_STRING);
        private CANcoder backCancoder = new CanAndMagSwerve(IntakeConstants.backCancoderID, IntakeConstants.CAN_STRING)

        private double targetAngleDegrees; 
        private double voltage;

        public Trigger atTargetAngle = new Trigger(() -> atTargetAngle()); // define

        public Intake() {
            configDevices(); // define

            if (Utils.isSimulation()) {
                configSimulation(); // define
            }

            targetAngleDegrees = getAngleDegrees(); // define
        }

        // configures everything and zeroes
        public void configDevides() {
            backMotor.getConfigurator().apply(IntakeConstants.backMotorConfig); // create

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

        public boolean atTargetAngle() {
            return Math.abs(getAngleDegrees() - targetAngleDegrees) < IntakeConstants.angleToleranceDegrees; // create
        }
        
        private void setAngleTargetDegrees(double targetAngleDegrees) {
            this.targetAngleDegrees = GremlinUtil.clampWithLogs(IntakeConstants.maxAngle, IntakeConstants.minAngle, targetAngleDegrees); // create
            
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
        
        public Command stow() {
            return goToAngleDegrees(IntakeConstants.minAngle); // create
        }

        public Command activateForCoral() {
            return goToAngleDegrees(IntakeConstants.maxAngle); // create
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
            updateMechanism2d(); // define

            SmartDashboard.putNumber("pivot angle", getAngleDegrees());
            SmartDashboard.putBoolean("rarely is the question asked, is our children spinning?", spinning());
        }


















        // sim
        private final FlywheelSim flywheelSim = new FlywheelSim(
            DCMotor.getKrakenX60Foc(0),
            IntakeConstants.flywheelGearing // create
        );
        private final SingleJointedArmSim singleJointedArmSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            IntakeConstants.pivotGearing, // create
            IntakeConstants.pivotMOI, // create
            IntakeConstants.intakeLength, // create 
            edu.wpi.first.math.util.Units.degreesToRadians(IntakeConstants.minAngle), // create
            edu.wpi.first.math.util.Units.degreesToRadians(IntakeConstants.maxAngle), // create
            true, 
            edu.wpi.first.math.util.Units.degreesToRadians(IntakeConstants.minAngle), // create 
            null);
        
        private TalonFXSimState frontMotorSim;
        private TalonFXSimState backMotorSim;
        private CANcoderSimState backCancoderSim;

        private Mechanism2d pivotMech = new Mechanism2d(IntakeConstants.canvasWidth, IntakeConstants.canvasHeight) // create x2
        
}
