package frc.robot;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;;

// more descriptive comments
public class PoseEstimate {

    public PhotonCamera limeLight;
    public RobotPoseEstimator robotPoseEstimator;
    private AprilTagFieldLayout apriltagLayout;

    public PoseEstimate(){
        // This exception accours if the `loadFromResource` fails
        // if that happens you should plan what the robot does and sees
        // meaning, that the getEstimatedGlobalPose would throw an exception or something
        try {
            apriltagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            limeLight = new PhotonCamera(VisionConstants.cameraName);

            var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
            camList.add(new Pair<PhotonCamera, Transform3d>(limeLight, VisionConstants.robotToCam));
    
            robotPoseEstimator = new RobotPoseEstimator(apriltagLayout, PoseStrategy.LOWEST_AMBIGUITY, camList);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Gets the last estimated position from vision
     * @param prevEstimatedRobotPose - The last estimated position to use as a reference.
     * @return The new estimated position using the latest vision 
     */
    public Pair<Pose3d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
        PhotonPipelineResult lastResult = limeLight.getLatestResult();

        if (result.isPresent() && lastResult.hasTargets() && lastResult.getBestTarget().getPoseAmbiguity() <= 0.2) {
            SmartDashboard.putNumber("Best target ambguitiy", limeLight.getLatestResult().getBestTarget().getPoseAmbiguity());
            // System.out.println("Latency: " + result.get().getSecond());
            double timestamp = (currentTime - (result.get().getSecond() / 1000));
            // System.out.println(timestamp);
            return new Pair<Pose3d, Double>(
                    result.get().getFirst(), timestamp);
        } else {
            return new Pair<Pose3d, Double>(null, 0.0);
        }
    }

    public AprilTagFieldLayout getApriltagLayout() {
        return apriltagLayout;
    }
    
}