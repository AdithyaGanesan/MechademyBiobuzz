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
    private final DcMotorEx flywheel;
    public double target_rpm = 0;

    //TODO: Create LookupTable class for shooting RPM based on distance
    //LookupTable pollenRPM = new LookupTable();

    public ShooterSubsystem(HardwareMap hwMap) {
        flywheel = hwMap.get(DcMotorEx.class, "shooter");
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //TODO: Add PID controllers
    public void setTargetVelocity(double target) {
        target_rpm = target;
        flywheel.setVelocity(rpmToTicksPerSecond(target));
    }

    public double getTargetVelocity() {return flywheel.getVelocity();}


    public void stop() {
        flywheel.setPower(0);
    }

    public boolean atTargetVelocity() {
        return Math.abs(flywheel.getVelocity() - rpmToTicksPerSecond(target_rpm)) < 50;
    }

    private double rpmToTicksPerSecond(double rpm) {
        // depends on your motor's ticks-per-rev — fill in once you know the motor
        return 0;
    }
}