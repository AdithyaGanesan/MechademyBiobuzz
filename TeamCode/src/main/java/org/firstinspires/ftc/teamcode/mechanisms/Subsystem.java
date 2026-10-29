package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Common shape for every mechanism. RobotHardware calls update() on each one every loop.
 * @author: AdithyaGanesan
 */
public interface Subsystem {
    /** Called every loop. Must never block (no sleep, no waiting loops). */
    void update();

    /** Called when the OpMode stops: cut power to motors and servos. */
    default void stop() {}

    /** Optional: add this subsystem's lines to telemetry. */
    default void addTelemetry(Telemetry telemetry) {}
}
