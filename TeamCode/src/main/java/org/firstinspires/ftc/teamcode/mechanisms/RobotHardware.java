package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Hardware map for a standard mecanum drivetrain.
 * Motor naming convention: front/back + left/right (e.g. "frontLeft").
 * Adjust config names to match your Driver Station robot configuration.
 * @author: AdithyqGanesan
 */
public class RobotHardware {

    // Drivetrain motors
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private final HardwareMap hwMap;

    public RobotHardware(HardwareMap hardwareMap) {
        this.hwMap = hardwareMap;
        init();
    }

    private void init() {
        // --- Drivetrain ---
        frontLeft  = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hwMap.get(DcMotorEx.class, "backLeft");
        backRight  = hwMap.get(DcMotorEx.class, "backRight");

        // Reverse one side so positive power drives forward on both sides.
        // Flip these if robot drives backward/sideways on init.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotorEx motor : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Field-relative-free (robot-centric) mecanum drive.
     * @param axial   forward/back (-1 to 1), positive = forward
     * @param lateral strafe (-1 to 1), positive = right
     * @param yaw     rotation (-1 to 1), positive = clockwise
     */
    public void driveMecanum(double axial, double lateral, double yaw) {
        double flPower = axial + lateral + yaw;
        double frPower = axial - lateral - yaw;
        double blPower = axial - lateral + yaw;
        double brPower = axial + lateral - yaw;

        // Normalize so no value exceeds 1.0
        double max = Math.max(1.0, Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                Math.max(Math.abs(blPower), Math.abs(brPower)))));

        frontLeft.setPower(flPower / max);
        frontRight.setPower(frPower / max);
        backLeft.setPower(blPower / max);
        backRight.setPower(brPower / max);
    }
}