package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Wrapper for a Limelight 3A smart camera.
 * Config name in Driver Station: "limelight"
 *
 * NOTE: written without hardware in hand
 * @author: AdithyaGanesan
 */
public class LimelightSubsystem {

    private final Limelight3A limelight;

    // Default poll rate (Hz). FTC SDK docs recommend 100 for competition use.
    private static final int POLL_RATE_HZ = 100;

    public LimelightSubsystem(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(POLL_RATE_HZ);
    }

    /** Call once at the start of an OpMode (after init, before start ideally). */
    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    /** Switch pipelines, e.g. 0 = AprilTag, 1 = color detection, etc. */
    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    /** Basic connectivity/status check. */
    public boolean isConnected() {
        LLStatus status = limelight.getStatus();
        return status != null;
    }

    /** Grabs the latest result. May return null if nothing valid yet. */
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    /**
     * Returns robot field pose from MegaTag localization, if the current
     * pipeline supports it and a valid result is available. Null otherwise.
     */
    public Pose3D getRobotPose() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            return result.getBotpose();
        }
        return null;
    }

    /** Horizontal offset from crosshair to target, in degrees. */
    public double getTx() {
        LLResult result = getLatestResult();
        return (result != null && result.isValid()) ? result.getTx() : 0.0;
    }

    /** Vertical offset from crosshair to target, in degrees. */
    public double getTy() {
        LLResult result = getLatestResult();
        return (result != null && result.isValid()) ? result.getTy() : 0.0;
    }

    /** Target area, 0-100% of image. */
    public double getTa() {
        LLResult result = getLatestResult();
        return (result != null && result.isValid()) ? result.getTa() : 0.0;
    }

    public boolean hasTarget() {
        LLResult result = getLatestResult();
        return result != null && result.isValid();
    }
}