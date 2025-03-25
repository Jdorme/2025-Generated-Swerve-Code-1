package frc.robot.subsystems;

import java.util.Optional;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
    // List of cameras
    private List<PhotonCamera> cameras = new ArrayList<>();
    private List<Transform3d> cameraTransforms = new ArrayList<>();
    private List<PhotonPoseEstimator> poseEstimators = new ArrayList<>();
    
    // Field layout
    private AprilTagFieldLayout fieldLayout;
    
    // Storage for the latest result from each camera with its strategy
    private static class CameraResult {
        public PhotonPipelineResult result;
        public PoseStrategy strategy;
        public int cameraIndex;
        
        public CameraResult(PhotonPipelineResult result, PoseStrategy strategy, int cameraIndex) {
            this.result = result;
            this.strategy = strategy;
            this.cameraIndex = cameraIndex;
        }
    }
    
    private List<CameraResult> latestResults = new ArrayList<>();
    
    // Configuration
    private boolean useMultipleAprilTags = true;
    private int preferredTagId = -1; // -1 means no preference

    public PhotonVisionSubsystem() {
        try {
            // Load the 2024 field layout
            fieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();
            
            // Add cameras - update names to match your configuration
            addCamera("EndGameCam", new Transform3d(
              new Translation3d(0.232029, -0.1676654, 0.3151886),  // X forward, Y left, Z up
              new Rotation3d(Math.toRadians(5), Math.toRadians(0),Math.toRadians( -104.9))  // Roll, pitch, yaw
          ));

            
            
            // You can add more cameras here
            // addCamera("SecondCamera", new Transform3d(...));
            
            // Initialize the results list with empty entries for each camera
            for (int i = 0; i < cameras.size(); i++) {
                latestResults.add(null);
            }
            
            SmartDashboard.putString("PhotonVision/Status", "Initialized with " + cameras.size() + " cameras");
        } catch (Exception e) {
            DriverStation.reportError("Error initializing PhotonVision: " + e.getMessage(), e.getStackTrace());
            SmartDashboard.putString("PhotonVision/Status", "Initialization Failed");
        }
    }
    
    private void addCamera(String cameraName, Transform3d robotToCamera) {
        try {
            // Create camera
            PhotonCamera camera = new PhotonCamera(cameraName);
            cameras.add(camera);
            cameraTransforms.add(robotToCamera);
            
            // Create pose estimator
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,  // Updated strategy enum
                robotToCamera
            );
            
            // Set multi-tag fallback strategy
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            
            poseEstimators.add(estimator);
            SmartDashboard.putString("PhotonVision/" + cameraName + "/Status", "Connected");
        } catch (Exception e) {
            DriverStation.reportError("Failed to add camera " + cameraName + ": " + e.getMessage(), e.getStackTrace());
            SmartDashboard.putString("PhotonVision/" + cameraName + "/Status", "Failed");
        }
    }
    
    @Override
    public void periodic() {
        try {
            // Update vision data for each camera
            for (int i = 0; i < cameras.size(); i++) {
                PhotonCamera camera = cameras.get(i);
                boolean resultUpdated = false;
                
                // Get the latest pipeline result (this now includes multi-tag data if available)
                PhotonPipelineResult result = camera.getLatestResult();
                
                // First try to use multi-tag processing if configured and available
                if (useMultipleAprilTags && result.hasTargets() && result.getTargets().size() > 1) {
                    latestResults.set(i, new CameraResult(
                        result,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        i
                    ));
                    
                    SmartDashboard.putString("PhotonVision/" + camera.getName() + "/ResultType", "MultiTag");
                    SmartDashboard.putNumber("PhotonVision/" + camera.getName() + "/TagCount", 
                        result.getTargets().size());
                    resultUpdated = true;
                }
                
                // If we didn't use multi-tag or it's not available, check for single-tag
                if (!resultUpdated && result.hasTargets()) {
                    // Check if we're looking for a specific tag
                    if (preferredTagId >= 0) {
                        boolean foundPreferredTag = false;
                        
                        // Look for the preferred tag
                        for (PhotonTrackedTarget target : result.getTargets()) {
                            if (target.getFiducialId() == preferredTagId) {
                                foundPreferredTag = true;
                                latestResults.set(i, new CameraResult(
                                    result,
                                    PoseStrategy.LOWEST_AMBIGUITY,
                                    i
                                ));
                                
                                SmartDashboard.putString("PhotonVision/" + camera.getName() + "/ResultType", 
                                                       "SingleTag (Preferred)");
                                SmartDashboard.putNumber("PhotonVision/" + camera.getName() + "/TagID", 
                                                       preferredTagId);
                                break;
                            }
                        }
                        
                        // If we didn't find the preferred tag, just use the best one
                        if (!foundPreferredTag && result.hasTargets()) {
                            latestResults.set(i, new CameraResult(
                                result,
                                PoseStrategy.LOWEST_AMBIGUITY,
                                i
                            ));
                            
                            SmartDashboard.putString("PhotonVision/" + camera.getName() + "/ResultType", 
                                                   "SingleTag (Best)");
                            SmartDashboard.putNumber("PhotonVision/" + camera.getName() + "/TagID", 
                                                   result.getBestTarget().getFiducialId());
                        }
                    } 
                    // No preferred tag, just use the best one
                    else if (result.hasTargets()) {
                        latestResults.set(i, new CameraResult(
                            result,
                            PoseStrategy.LOWEST_AMBIGUITY,
                            i
                        ));
                        
                        SmartDashboard.putString("PhotonVision/" + camera.getName() + "/ResultType", 
                                               "SingleTag");
                        SmartDashboard.putNumber("PhotonVision/" + camera.getName() + "/TagID", 
                                               result.getBestTarget().getFiducialId());
                    }
                } else if (!resultUpdated) {
                    // No targets detected
                    SmartDashboard.putString("PhotonVision/" + camera.getName() + "/ResultType", "No Targets");
                }
            }
        } catch (Exception e) {
            DriverStation.reportError("Error in PhotonVision periodic: " + e.getMessage(), e.getStackTrace());
        }
    }
    
    /**
     * Set whether to use multiple AprilTags or prefer a single tag
     */
    public void setUseMultipleAprilTags(boolean useMultiple) {
        this.useMultipleAprilTags = useMultiple;
        SmartDashboard.putBoolean("PhotonVision/UseMultiTags", useMultiple);
    }
    
    /**
     * Set a preferred AprilTag ID to use for pose estimation
     * @param tagId The preferred tag ID, or -1 to use any visible tag
     */
    public void setPreferredAprilTag(int tagId) {
        this.preferredTagId = tagId;
        SmartDashboard.putNumber("PhotonVision/PreferredTagID", tagId);
    }
    
    /**
     * Get all estimated global poses from vision
     * @return A list of estimated poses, one per camera
     */
    public List<EstimatedRobotPose> getEstimatedGlobalPoses() {
        List<EstimatedRobotPose> poses = new ArrayList<>();
        
        // Process each camera's latest result
        for (int i = 0; i < latestResults.size(); i++) {
            CameraResult cameraResult = latestResults.get(i);
            
            // Skip if no result for this camera
            if (cameraResult == null) {
                continue;
            }
            
            try {
                // Use the pose estimator to get the field-relative pose
                PhotonPoseEstimator estimator = poseEstimators.get(cameraResult.cameraIndex);
                estimator.setPrimaryStrategy(cameraResult.strategy);
                
                Optional<EstimatedRobotPose> estimatedPose = estimator.update(cameraResult.result);
                
                if (estimatedPose.isPresent()) {
                    poses.add(estimatedPose.get());
                }
            } catch (Exception e) {
                DriverStation.reportError("Error processing pose for camera " + i + ": " + e.getMessage(), e.getStackTrace());
            }
        }
        
        SmartDashboard.putNumber("PhotonVision/ValidPoseCount", poses.size());
        return poses;
    }
    
    /**
     * Get the best estimated pose based on confidence calculations
     * @return The pose with the highest confidence, or empty if no poses available
     */
    public Optional<EstimatedRobotPose> getBestEstimatedPose() {
        List<EstimatedRobotPose> poses = getEstimatedGlobalPoses();
        
        if (poses.isEmpty()) {
            return Optional.empty();
        }
        
        EstimatedRobotPose bestPose = null;
        double bestConfidence = -1;
        String cameraBestPose = "";
        
        // Find the pose with the highest confidence
        for (int i = 0; i < poses.size(); i++) {
            double confidence = calculatePoseConfidence(poses.get(i));
            
            if (confidence > bestConfidence) {
                bestConfidence = confidence;
                bestPose = poses.get(i);
                //cameraBestPose = cameras.get(latestResults.get(i).cameraIndex).getName();

            }
        }
        
        // Log the best confidence
        if (bestPose != null) {
            SmartDashboard.putNumber("PhotonVision/BestPoseConfidence", bestConfidence);
            SmartDashboard.putString("PhotonVision/BestPoseStrategy", 
                                    bestPose.strategy.toString());
            SmartDashboard.putNumber("PhotonVision/BestPoseTagCount", 
                                    bestPose.targetsUsed.size());
            SmartDashboard.putString("PhotonVision/BestPose", 
                                    String.format("(%.2f, %.2f, %.2f)", 
                                                 bestPose.estimatedPose.getX(), 
                                                 bestPose.estimatedPose.getY(),
                                                 bestPose.estimatedPose.getRotation().getZ()
                                                 ));

            //SmartDashboard.putString("PhotonVision/BestCamera", cameraBestPose);
        }
        
        return Optional.ofNullable(bestPose);
    }
    
    /**
     * Calculate the confidence of a pose estimate based on number of tags and distances
     * @param pose The estimated robot pose
     * @return A confidence value from 0.0 to 1.0
     */
    public double calculatePoseConfidence(EstimatedRobotPose pose) {
        List<PhotonTrackedTarget> targetsUsed = pose.targetsUsed;
        
        // No targets = no confidence
        if (targetsUsed.isEmpty()) {
            return 0.0;
        }
        
        // More tags = more confidence
        int tagCount = targetsUsed.size();
        
        // Calculate average distance - closer tags = more confidence
        double avgDistance = 0.0;
        for (PhotonTrackedTarget target : targetsUsed) {
            avgDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        avgDistance /= tagCount;
        // Multi-tag strategies get a bonus
        double strategyBonus = pose.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR ? 0.2 : 0.0;
        
        // Calculate confidence based on tag count and distance
        double distanceConfidence = Math.max(0.1, Math.min(1.0, 1.1 / avgDistance));
        double tagCountConfidence = Math.min(1.0, tagCount * 0.5); // 2+ tags = full confidence
        
        // Combine factors with appropriate weights
        double confidence = 0.4 * distanceConfidence + 0.6 * tagCountConfidence + strategyBonus;
        
        // Cap at 1.0
        return Math.min(1.0, confidence);
    }
    
    /**
     * Checks if there are any valid pose estimates available
     * @return true if at least one valid pose estimate is available
     */
    public boolean hasPoseEstimate() {
        for (CameraResult result : latestResults) {
            if (result != null) {
                return true;
            }
        }
        return false;
    }
}