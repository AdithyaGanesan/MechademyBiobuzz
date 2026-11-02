package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Robot pose from the goBILDA Pinpoint (two 4-bar pods + its own IMU), via the FTC SDK's driver.
 * Pose is in the HiveState / ShotSolver field frame: inches, origin at field center, +X toward blue,
 * +Y away from the audience, heading degrees counterclockwise from +X. The Pinpoint tracks in whatever
 * frame its starting pose was given in, so auton must call setPose() with the real starting position.
 * Temporary until Pedro's Pinpoint localizer is set up; then this reads the pose from Pedro instead.
 * @author: AdithyaGanesan
 */
public class Odometry implements Subsystem {
    // From the CAD (ROBOT12): the left pod tracks forward, the right pod tracks sideways.
    // X_POD_OFFSET = how far left (+) or right (-) of center the forward-tracking pod is.
    // Y_POD_OFFSET = how far forward (+) or behind (-) center the sideways-tracking pod is.
    // Re-measure on the built robot.
    private static final double X_POD_OFFSET_IN = 4.24;
    private static final double Y_POD_OFFSET_IN = -5.39;

    private final GoBildaPinpointDriver pinpoint;

    public Odometry(HardwareMap hwMap) {
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(X_POD_OFFSET_IN, Y_POD_OFFSET_IN, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //TODO: Check on the robot: pushing forward should raise X, pushing left should raise Y.
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        // No reset here: TeleOp keeps the pose auton ended with, as long as the robot stays powered.
    }

    /** Auton INIT: tell the Pinpoint where the robot starts (field frame, inches, degrees). */
    public void setPose(double x, double y, double headingDeg) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, headingDeg));
    }

    public double getX() {return pinpoint.getPosX(DistanceUnit.INCH);}

    public double getY() {return pinpoint.getPosY(DistanceUnit.INCH);}

    public double getHeadingDeg() {return pinpoint.getHeading(AngleUnit.DEGREES);}

    /** Reads new position data from the Pinpoint. Must run every loop. */
    @Override
    public void update() {
        pinpoint.update();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Robot pose (in, °)", "x %.1f  y %.1f  h %.1f", getX(), getY(), getHeadingDeg());
    }
}
