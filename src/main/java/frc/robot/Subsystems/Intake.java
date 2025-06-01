package frc.robot.Subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
        @SuppressWarnings("unused")
        private TalonFX frontMotor = new TalonFX(IntakeConstants.frontMotorID, IntakeConstants.CAN_STRING);
        private TalonFX backMotor = new TalonFX(IntakeConstants.backMotorID, IntakeConstants.CAN_STRING);

        private double targetAngleDegrees; 

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
            frontMotor.getConfigurator().apply()
        }
}
