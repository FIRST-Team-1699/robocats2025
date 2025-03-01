package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;

public class PhotonCameraManager {
    public Transform3d robotToCamera;

    private PhotonCamera camera;
    private Matrix<N3,N1> currentStandardDeviation;
    private Optional<PhotonPipelineResult> latestResult;

    private PhotonPoseEstimator poseEstimator;

    public PhotonCameraManager(
        Transform3d robotToCamera, PhotonCamera camera, Matrix<N3, N1> currentStandardDeviation, PoseStrategy primaryStategy, PoseStrategy fallbackStrategy) {
        this.robotToCamera = robotToCamera;
        this.camera = camera;
        this.currentStandardDeviation = currentStandardDeviation;
        this.poseEstimator = new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout, primaryStategy, robotToCamera);
        poseEstimator.setMultiTagFallbackStrategy(fallbackStrategy);
        this.latestResult = Optional.empty();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        // CHECKING DATA WITH CAMERA ONE
        for (var change : camera.getAllUnreadResults()) {
            visionEst = poseEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
            latestResult = Optional.of(change);
        }
        
        return visionEst;
    }

    public void setStrategy(PoseStrategy primaryStrategy) {
        poseEstimator.setPrimaryStrategy(primaryStrategy);
    }

    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // USES A 1 by 3 (1 ROW, 3 COLUM) MATRIX TO STORE DATA
            currentStandardDeviation = VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
                for (var tgt : targets) {
                    var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                    if (tagPose.isEmpty()) continue;
                        numTags++;
                        avgDist +=
                            tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                }

                if (numTags == 0) {
            // No tags visible. Default to single-tag std devs
                    currentStandardDeviation = VisionConstants.kSingleTagStdDevs;
                } else {
            // One or more tags visible, run the full heuristic.
                    avgDist /= numTags;
            // Decrease std devs if multiple targets are visible
                    if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
            // Increase std devs based on (average) distance
                    if (numTags == 1 && avgDist > 4)
                        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                        currentStandardDeviation = estStdDevs;
            }
        }
    }

    public Optional<PhotonPipelineResult> getLatestResult() {
        if(latestResult.isEmpty()) {
            
        }
        else {

        }
        //TODO: Iderate through list(Array/List/LinkedList) and return values
        return null;
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return currentStandardDeviation;
    }

    public void setPoseStrategy(PoseStrategy primaryPose, PoseStrategy fallbackPoseStrategy) {
        poseEstimator.setLastPose(poseEstimator.getReferencePose());

        poseEstimator.setPrimaryStrategy(primaryPose);
        poseEstimator.setMultiTagFallbackStrategy(fallbackPoseStrategy);
    }

}
