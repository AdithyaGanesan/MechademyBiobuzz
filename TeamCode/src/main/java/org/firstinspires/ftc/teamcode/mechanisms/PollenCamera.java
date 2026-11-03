package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hivevision.BallBlobPipeline;
import org.firstinspires.ftc.teamcode.hivevision.BallDetectorPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * Pollen camera: Logitech C270 at the front of the robot, tilted down, running Hive Vision's color pipeline.
 * Turns the best yellow ball on screen into a spot on the floor relative to the robot.
 * NOTE: written without hardware in hand
 * @author: AdithyaGanesan
 */
public class PollenCamera implements Subsystem {
    // Mount from the ROBOT12 CAD (drawn as a Thriftiest Cam; the C270 is assumed to use the same spot).
    private static final double CAMERA_FORWARD_IN = 8.3;
    private static final double CAMERA_HEIGHT_IN = 7.3;
    private static final double CAMERA_PITCH_DOWN_DEG = 30;
    // C270 is ~55° diagonal; at 1280x720 that works out to roughly these. Check against a real image.
    private static final double HFOV_DEG = 48.8;
    private static final double VFOV_DEG = 28.6;
    // Pollen is ~2.8" across, so its center sits ~1.4" off the floor.
    private static final double POLLEN_CENTER_HEIGHT_IN = 1.4;

    /** Where a ball is, relative to the robot's center: inches forward and inches left. */
    public static final class Spot {
        public final double forwardIn, leftIn;
        Spot(double forwardIn, double leftIn) {this.forwardIn = forwardIn; this.leftIn = leftIn;}
    }

    // Hive Vision tunes its size gates at 1280x720, so run the C270 at that resolution.
    private final BallDetectorPipeline pipeline = new BallDetectorPipeline();
    private final VisionPortal portal;
    private Spot lastSpot = null;

    /** @param viewportId from VisionPortal.makeMultiPortalView(), so both cameras can share the live view */
    public PollenCamera(HardwareMap hwMap, int viewportId) {
        portal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "pollenCam"))
                .setCameraResolution(new Size(1280, 720))
                .setLiveViewContainerId(viewportId)
                .addProcessor(pipeline)
                .build();
    }

    /** @return the most ball-like pollen in view, or null if none */
    public Spot getPollen() {return lastSpot;}

    /** Turn pollen detection on or off; off saves Control Hub CPU when nothing needs it. */
    public void setEnabled(boolean enabled) {
        portal.setProcessorEnabled(pipeline, enabled);
    }

    @Override
    public void update() {
        BallBlobPipeline.BallBlob ball = pipeline.bestOf(BallBlobPipeline.BallColor.YELLOW);
        lastSpot = (ball == null) ? null : toFloor(ball.cxNorm, ball.cyNorm);
    }

    /**
     * Screen position (0..1 across, 0..1 down) to floor position. Builds the ray through that pixel,
     * tilts it down by the camera pitch, and follows it until it drops to pollen-center height.
     */
    static Spot toFloor(double cxNorm, double cyNorm) {
        double right = (cxNorm - 0.5) * 2 * Math.tan(Math.toRadians(HFOV_DEG / 2));   // per 1 unit along the lens
        double down = (cyNorm - 0.5) * 2 * Math.tan(Math.toRadians(VFOV_DEG / 2));
        double p = Math.toRadians(CAMERA_PITCH_DOWN_DEG);
        double forwardWorld = Math.cos(p) - down * Math.sin(p);
        double downWorld = Math.sin(p) + down * Math.cos(p);
        if (downWorld <= 0) return null;   // ray points at or above the horizon: not on the floor
        double t = (CAMERA_HEIGHT_IN - POLLEN_CENTER_HEIGHT_IN) / downWorld;
        return new Spot(CAMERA_FORWARD_IN + t * forwardWorld, -t * right);
    }

    @Override
    public void stop() {
        portal.close();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        if (lastSpot == null) telemetry.addData("Pollen", "none seen");
        else telemetry.addData("Pollen (in)", "fwd %.1f  left %.1f", lastSpot.forwardIn, lastSpot.leftIn);
    }
}
