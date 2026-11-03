package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Intake: one motor belted to the front roller and the lower conveyor that carries pollen back to the transfer.
 * CAD: goBILDA 5000 Series bare motor (~5800 RPM) at 1:1, so powers start low; raise them if it's too slow,
 * or swap to a geared motor and retune. Rule G407: never hold more than 4 pollen; no sensor counts them yet.
 * NOTE: written without hardware in hand
 * @author: AdithyaGanesan
 */
public class Intake implements Subsystem {
    // PLACEHOLDERS, tune on the robot.
    private static final double IN_POWER = 0.6;
    private static final double OUT_POWER = -0.6;

    public enum State {OFF, IN, OUT}

    private final DcMotorEx motor;
    private State state = State.OFF;

    public Intake(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "intake");
        //TODO: Check on the robot that positive power pulls pollen in.
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void in() {state = State.IN;}

    /** Spits pollen back out, e.g. to clear a jam or drop a 5th ball. */
    public void out() {state = State.OUT;}

    public void off() {state = State.OFF;}

    public State getState() {return state;}

    @Override
    public void update() {
        switch (state) {
            case IN:  motor.setPower(IN_POWER); break;
            case OUT: motor.setPower(OUT_POWER); break;
            default:  motor.setPower(0);
        }
    }

    @Override
    public void stop() {
        state = State.OFF;
        motor.setPower(0);
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", state);
    }
}
