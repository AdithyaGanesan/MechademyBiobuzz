package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Wrapper for a shooter mechanism
 * NOTE: written without hardware in hand
 * @author: AdithyaGanesan
 */
public class ShooterSubsystem {
    // goBILDA 5000 Series motor (5000-0002-4008): 28 encoder ticks per motor rev, ~5800 RPM free speed.
    private static final double TICKS_PER_MOTOR_REV = 28;
    // CAD: 20T motor pulley to 20T flywheel pulley, so the flywheel turns once per motor rev.
    private static final double MOTOR_REVS_PER_FLYWHEEL_REV = 1.0;
    private static final double TICKS_PER_FLYWHEEL_REV = TICKS_PER_MOTOR_REV * MOTOR_REVS_PER_FLYWHEEL_REV;
    private static final double RPM_TOLERANCE = 100;

    private final DcMotorEx flywheel;
    private double target_rpm = 0;

    //TODO: Create LookupTable class for shooting RPM based on distance
    //LookupTable pollenRPM = new LookupTable();

    public ShooterSubsystem(HardwareMap hwMap) {
        flywheel = hwMap.get(DcMotorEx.class, "shooter");
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // setVelocity() only holds a speed when the motor runs in encoder (velocity PID) mode.
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //TODO: Add PID controllers
    /** @param target flywheel speed in RPM */
    public void setTargetVelocity(double target) {
        target_rpm = target;
        flywheel.setVelocity(rpmToTicksPerSecond(target));
    }

    /** @return the flywheel speed we asked for, in RPM */
    public double getTargetVelocity() {return target_rpm;}

    /** @return the flywheel speed the encoder measures right now, in RPM */
    public double getVelocity() {return ticksPerSecondToRpm(flywheel.getVelocity());}


    public void stop() {
        target_rpm = 0;
        flywheel.setPower(0);
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
