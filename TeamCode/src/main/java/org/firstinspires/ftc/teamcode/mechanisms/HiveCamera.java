package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Hive camera: ThriftyBot Thriftiest Cam on the back of the turret, tilted up, looking the way the shooter fires.
 * Its only job is to tell HiveState which cell of our hive faces up.
 *
 * Each cell has a cluster of 4 AprilTags on its bottom (SDK clusters: RED SCORING 30-33, RED AUDIENCE 34-37,
 * BLUE AUDIENCE 38-41, BLUE SCORING 42-45; SCORING = the far-side cell). The up cell's cluster sits about 49.7"
 * above the floor and the down cell's about 35.7", so the height of any of our clusters says which cell is up.
 * The turret's rotation doesn't change that height, only the camera's tilt does.
 * NOTE: written without hardware in hand
 * @author: AdithyaGanesan
 */
public class HiveCamera implements Subsystem {
    // From the ROBOT12 CAD. Re-measure on the built robot.
    private static final double CAMERA_HEIGHT_IN = 13.5;
    private static final double CAMERA_PITCH_UP_DEG = 40;

    // Up-cell cluster ~49.7", down-cell cluster ~35.7" (field CAD); split the difference.
    private static final double UP_CELL_MIN_HEIGHT_IN = 42.7;
    // Same answer this many frames in a row before telling HiveState, so one bad frame can't flip the target.
    private static final int FRAMES_TO_CONFIRM = 5;

    private final AprilTagProcessor aprilTags;
    private final VisionPortal portal;
    private final HiveState hiveState;
    private final String ourPrefix;

    private HiveState.Cell candidate = null;
    private int candidateFrames = 0;
    private double lastHeightIn = Double.NaN;
    private String lastCluster = "none";

    /** @param viewportId from VisionPortal.makeMultiPortalView(), so both cameras can share the live view */
    public HiveCamera(HardwareMap hwMap, HiveState hiveState, HiveState.Alliance alliance, int viewportId) {
        this.hiveState = hiveState;
        this.ourPrefix = alliance.name();   // cluster names start with "RED" or "BLUE"

        aprilTags = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        //TODO: Calibrate the Thriftiest Cam (lens intrinsics) at this resolution for accurate distances.
        portal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "hiveCam"))
                .setCameraResolution(new Size(1280, 800))
                .setLiveViewContainerId(viewportId)
                .addProcessor(aprilTags)
                .build();
    }

    /** Checks new frames for our hive's tag clusters and updates HiveState once an answer holds steady. */
    @Override
    public void update() {
        for (AprilTagDetection d : aprilTags.getFreshDetections()) {
            if (!(d instanceof AprilTagClusterDetection) || d.ftcPose == null) continue;
            String name = ((AprilTagClusterDetection) d).metadata.name;
            if (!name.startsWith(ourPrefix)) continue;

            double heightIn = clusterHeightIn(d.ftcPose.y, d.ftcPose.z);
            HiveState.Cell cluster = name.endsWith("AUDIENCE") ? HiveState.Cell.AUDIENCE : HiveState.Cell.FAR;
            HiveState.Cell other = (cluster == HiveState.Cell.FAR) ? HiveState.Cell.AUDIENCE : HiveState.Cell.FAR;
            // A high cluster belongs to the up cell; a low one means the other cell is up.
            HiveState.Cell up = (heightIn > UP_CELL_MIN_HEIGHT_IN) ? cluster : other;

            lastHeightIn = heightIn;
            lastCluster = name;
            confirm(up);
        }
    }

    /**
     * Camera reports the cluster as y inches out along the lens and z inches up in the camera's own frame.
     * Tilting that frame up by the camera pitch turns it into height above the camera.
     */
    static double clusterHeightIn(double yOut, double zUp) {
        double p = Math.toRadians(CAMERA_PITCH_UP_DEG);
        return CAMERA_HEIGHT_IN + yOut * Math.sin(p) + zUp * Math.cos(p);
    }

    private void confirm(HiveState.Cell up) {
        if (up == candidate) {
            candidateFrames++;
        } else {
            candidate = up;
            candidateFrames = 1;
        }
        if (candidateFrames == FRAMES_TO_CONFIRM && up != hiveState.getUpCell()) {
            hiveState.setFromCamera(up);
        }
    }

    @Override
    public void stop() {
        portal.close();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Hive cam", "%s at %.1f in (%s, %d frames)", lastCluster, lastHeightIn,
                candidate == null ? "no answer yet" : candidate.name(), candidateFrames);
    }
}
