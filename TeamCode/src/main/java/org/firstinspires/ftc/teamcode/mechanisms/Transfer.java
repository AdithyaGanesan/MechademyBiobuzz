package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Transfer: one motor driving the upper rollers that push pollen up through the turret into the shooter.
 * It only runs while feeding shots; otherwise it brakes so pollen waits below the flywheel.
 * CAD: goBILDA 5000 Series bare motor (~5800 RPM) at 1:1, so powers start low.
 * NOTE: written without hardware in hand
 * @author: AdithyaGanesan
 */
public class Transfer implements Subsystem {
    // PLACEHOLDERS, tune on the robot.
    private static final double FEED_POWER = 0.5;
    private static final double REVERSE_POWER = -0.4;

    public enum State {OFF, FEED, REVERSE}

    private final DcMotorEx motor;
    private State state = State.OFF;

    public Transfer(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "transfer");
        //TODO: Check on the robot that positive power moves pollen up toward the shooter.
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        // Brake when off so pollen stays put instead of drifting into the flywheel.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Push pollen up into the shooter. Callers should only do this when RobotHardware.readyToShoot(). */
    public void feed() {state = State.FEED;}

    /** Back pollen down, e.g. to clear a jam. */
    public void reverse() {state = State.REVERSE;}

    public void off() {state = State.OFF;}

    public State getState() {return state;}

    @Override
    public void update() {
        switch (state) {
            case FEED:    motor.setPower(FEED_POWER); break;
            case REVERSE: motor.setPower(REVERSE_POWER); break;
            default:      motor.setPower(0);
        }
    }

    @Override
    public void stop() {
        state = State.OFF;
        motor.setPower(0);
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Transfer", state);
    }
}
