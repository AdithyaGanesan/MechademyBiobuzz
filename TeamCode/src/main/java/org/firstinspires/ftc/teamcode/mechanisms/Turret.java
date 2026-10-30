package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turret: goBILDA 5203 435 RPM motor, 32T pinion driving a 160T ring gear (5:1).
 * Angles are robot-relative degrees: 0 = facing the robot's front, positive = counterclockwise seen from above.
 * The team points the turret forward by hand before a match; auton calls zeroHere(), TeleOp does not,
 * because the encoder keeps counting between OpModes until it's reset or the robot powers off.
 * NOTE: written without hardware in hand
 * @author: AdithyaGanesan
 */
public class Turret implements Subsystem {
    // 5203 435 RPM: 384.5 encoder ticks per output rev. 160T ring / 32T pinion = 5 motor revs per turret rev.
    private static final double TICKS_PER_MOTOR_REV = 384.5;
    private static final double MOTOR_REVS_PER_TURRET_REV = 160.0 / 32.0;
    private static final double TICKS_PER_DEGREE = TICKS_PER_MOTOR_REV * MOTOR_REVS_PER_TURRET_REV / 360.0;

    // Cable chain allows ±360°. Stay a little inside it.
    private static final double LIMIT_DEG = 355;

    // PLACEHOLDERS, tune on the robot. Power per degree of error, and damping per degree/second of speed.
    private static final double KP = 0.02;
    private static final double KD = 0.0005;
    private static final double MAX_POWER = 0.8;
    private static final double TOLERANCE_DEG = 2;

    private final DcMotorEx motor;
    private double targetDeg = 0;

    public Turret(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "turret");
        //TODO: Check on the robot that positive power turns the turret counterclockwise (seen from above).
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // No encoder reset here: TeleOp must keep the angle auton left off at.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetDeg = getAngle();
    }

    /** Call with the turret pointing straight forward (auton INIT, or a driver re-zero button). */
    public void zeroHere() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetDeg = 0;
    }

    /** @return current turret angle in degrees, 0 = forward, counterclockwise positive, can go past ±180 */
    public double getAngle() {
        return motor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    /**
     * Point the turret at a robot-relative direction. Any equivalent angle works (90 and -270 are the same
     * direction); this picks the one within the cable limits that is closest to where the turret is now,
     * which also unwinds the cable when the short way would pass a limit.
     */
    public void setTargetAngle(double degrees) {
        targetDeg = chooseTarget(degrees, getAngle(), LIMIT_DEG);
    }

    public double getTargetAngle() {return targetDeg;}

    public boolean atTarget() {
        return Math.abs(targetDeg - getAngle()) < TOLERANCE_DEG;
    }

    static double chooseTarget(double desiredDeg, double currentDeg, double limitDeg) {
        double base = desiredDeg - 360.0 * Math.round(desiredDeg / 360.0);   // -180..180
        double best = Double.NaN;
        for (int k = -2; k <= 2; k++) {
            double candidate = base + 360.0 * k;
            if (Math.abs(candidate) > limitDeg) continue;
            if (Double.isNaN(best) || Math.abs(candidate - currentDeg) < Math.abs(best - currentDeg)) best = candidate;
        }
        return best;
    }

    /** PD control toward the target. Runs every loop. */
    @Override
    public void update() {
        double error = targetDeg - getAngle();
        double speedDegPerSec = motor.getVelocity() / TICKS_PER_DEGREE;
        double power = KP * error - KD * speedDegPerSec;
        motor.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret angle", "%.1f° / %.1f°", getAngle(), targetDeg);
        telemetry.addData("Turret on target", atTarget());
    }
}
