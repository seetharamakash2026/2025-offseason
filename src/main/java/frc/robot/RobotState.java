package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {
    private DriveState driveState;
    private static RobotState robotState;

    public enum DriveState {
        TELEOP, // update smartdashboard below
        INTAKE
    }

    private RobotState() {
        this.driveState = DriveState.TELEOP;
    }

    public static RobotState getRobotState() {
        if (robotState == null) {
            robotState = new RobotState();
        }
        return robotState;
    }

    public DriveState getDriveState() {
        return driveState;
    }
    public void setDriveState(DriveState newDriveState) {
        this.driveState = newDriveState;
        SmartDashboard.putBoolean("Teleop State", getDriveState() == DriveState.TELEOP);
        SmartDashboard.putBoolean("Intake State", getDriveState() == DriveState.INTAKE);
    } 
    
}
