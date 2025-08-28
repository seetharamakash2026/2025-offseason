package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.CommandSwerveDrivetrain;

public class LimelightCamera {
    private static CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
    private String cameraName;
    private Pose3d relRobotPose;

    public LimelightCamera(String CameraName, int cameraSettingsIndex) {
        cameraName = CameraName;
        LimelightHelpers.setCameraPose_RobotSpace(
            CameraName,
            relRobotPose.getZ(),
            relRobotPose.getX(),
            relRobotPose.getY(),
            relRobotPose.getRotation().getZ(),
            relRobotPose.getRotation().getY(),
            relRobotPose.getRotation().getX()
        );
    }

    public void gather() {
        LimelightHelpers.PoseEstimate poseEstimate;
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
            } else {
                poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed(cameraName);
            }
        } else {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed(cameraName);
        }

        drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
    }
}