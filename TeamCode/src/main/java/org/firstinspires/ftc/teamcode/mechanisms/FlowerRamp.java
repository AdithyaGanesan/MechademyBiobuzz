package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Flower ramps: two goBILDA servos, one on each side wall, driving the ramps through pulleys.
 * They always move together; the right servo is mirrored so the same position means the same ramp angle.
 * Used at any point in the match to release pollen from a flower.
 * NOTE: written without hardware in hand
 * @author: AdithyaGanesan
 */
public class FlowerRamp implements Subsystem {
    // PLACEHOLDERS, set on the robot: servo positions for the ramp up and down.
    private static final double UP_POS = 0.0;
    private static final double DOWN_POS = 0.5;

    private final Servo left, right;
    private boolean up = true;

    public FlowerRamp(HardwareMap hwMap) {
        left = hwMap.get(Servo.class, "rampLeft");
        right = hwMap.get(Servo.class, "rampRight");
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.REVERSE);
        // No position is sent here, so the ramps don't move during INIT.
    }

    public void up() {
        up = true;
        setPosition(UP_POS);
    }

    public void down() {
        up = false;
        setPosition(DOWN_POS);
    }

    public void toggle() {
        if (up) down(); else up();
    }

    public boolean isUp() {return up;}

    private void setPosition(double position) {
        left.setPosition(position);
        right.setPosition(position);
    }

    /** Nothing to do each loop: servos hold their own position. */
    @Override
    public void update() {}

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Flower ramp", up ? "UP" : "DOWN");
    }
}
