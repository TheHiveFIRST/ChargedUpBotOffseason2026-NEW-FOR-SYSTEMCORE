
package first.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;

    public VisionSubsystem() {
        camera = new PhotonCamera("Arducam");
    }

    @Override
    public void periodic() {
        // Get one camera result per cycle
        latestResult = camera.getLatestResult();

        SmartDashboard.putBoolean("Has Target", hasTarget());
        SmartDashboard.putNumber("Target Yaw", getYaw());
        SmartDashboard.putNumber("Tag ID", getTagID());
    }

    public boolean hasTarget() {
        return latestResult != null && latestResult.hasTargets();
    }

    public double getYaw() {
        if (!hasTarget()) {
            return 0.0;
        }

        PhotonTrackedTarget target = latestResult.getBestTarget();
        return target.getYaw();
    }

    public int getTagID() {
        if (!hasTarget()) {
            return -1;
        }

        return latestResult.getBestTarget().getFiducialId();
    }
}
