package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Wrapper for a shooter mechanism
 * NOTE: written without hardware in hand
 * @author: AdithyaGanesan
 */
public class ShooterSubsystem implements Subsystem {
    // goBILDA 5000 Series motor (5000-0002-4008): 28 encoder ticks per motor rev, ~5800 RPM free speed.
    private static final double TICKS_PER_MOTOR_REV = 28;
    private static final double MOTOR_FREE_RPM = 5800;
    // CAD: 20T motor pulley to 20T flywheel pulley, so the flywheel turns once per motor rev.
    private static final double MOTOR_REVS_PER_FLYWHEEL_REV = 1.0;
    private static final double TICKS_PER_FLYWHEEL_REV = TICKS_PER_MOTOR_REV * MOTOR_REVS_PER_FLYWHEEL_REV;
    // Stay a little under free speed so the velocity controller always has headroom.
    private static final double MAX_RPM = 5500;
    private static final double RPM_TOLERANCE = 100;

    // Velocity PIDF starting point (goBILDA recipe): F = 32767 / max ticks per second, P = F/10, I = P/10.
    // Tune on the real robot.
    private static final double F = 32767 / (MOTOR_FREE_RPM * TICKS_PER_MOTOR_REV / 60.0);
    private static final double P = F / 10;
    private static final double I = P / 10;
    private static final double D = 0;

    // Hood: SWYFT servo (270° range) turns a 35T gear driving a 218T gear segment around the flywheel axle.
    private static final double SERVO_RANGE_DEG = 270;
    private static final double SERVO_DEG_PER_HOOD_DEG = 218.0 / 35.0;
    // PLACEHOLDERS, measure on the robot: servo position with the hood at its lowest point, and the
    // highest hood angle before the 22-tooth segment runs out. Going past the end can strip the gear.
    private static final double HOOD_ZERO_POS = 0.0;
    private static final double HOOD_MAX_DEG = 30;

    private final DcMotorEx flywheel;
    private final Servo hood;
    private double target_rpm = 0;
    private double hoodAngleDeg = 0;

    // RPM and hood angle by distance come from ShotSolver's LookupTables.

    public ShooterSubsystem(HardwareMap hwMap) {
        flywheel = hwMap.get(DcMotorEx.class, "shooter");
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // setVelocity() only holds a speed when the motor runs in encoder (velocity PID) mode.
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

        // No position is sent here, so the hood doesn't move during INIT.
        hood = hwMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.FORWARD);
    }

    /** @param degrees hood angle above its lowest position, clamped to 0..HOOD_MAX_DEG */
    public void setHoodAngle(double degrees) {
        hoodAngleDeg = Range.clip(degrees, 0, HOOD_MAX_DEG);
        hood.setPosition(HOOD_ZERO_POS + hoodAngleDeg * SERVO_DEG_PER_HOOD_DEG / SERVO_RANGE_DEG);
    }

    /** @return the hood angle we last asked for, in degrees */
    public double getHoodAngle() {return hoodAngleDeg;}

    /** Nothing to do each loop: the hub runs the flywheel's velocity PID and the servo holds itself. */
    @Override
    public void update() {}

    /** @param target flywheel speed in RPM, capped at MAX_RPM */
    public void setTargetVelocity(double target) {
        target_rpm = Range.clip(target, 0, MAX_RPM);
        flywheel.setVelocity(rpmToTicksPerSecond(target_rpm));
    }

    /** @return the flywheel speed we asked for, in RPM */
    public double getTargetVelocity() {return target_rpm;}

    /** @return the flywheel speed the encoder measures right now, in RPM */
    public double getVelocity() {return ticksPerSecondToRpm(flywheel.getVelocity());}


    /** Lets the flywheel coast down. The hood servo keeps its position so the hood doesn't drop. */
    @Override
    public void stop() {
        target_rpm = 0;
        flywheel.setPower(0);
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter RPM", "%.0f / %.0f", getVelocity(), target_rpm);
        telemetry.addData("Shooter at speed", atTargetVelocity());
        telemetry.addData("Hood angle", "%.1f°", hoodAngleDeg);
    }

    public boolean atTargetVelocity() {
        return Math.abs(getVelocity() - target_rpm) < RPM_TOLERANCE;
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * TICKS_PER_FLYWHEEL_REV / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / TICKS_PER_FLYWHEEL_REV;
    }
}
