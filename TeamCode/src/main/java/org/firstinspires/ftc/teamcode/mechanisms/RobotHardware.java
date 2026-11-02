package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

/**
 * Hardware map for a standard mecanum drivetrain.
 * Naming convention: front/back + Left/Right (e.g. "frontLeft").
 * Adjust config names to match your Driver Station robot configuration.
 * @author: AdithyqGanesan
 */
public class RobotHardware {

    // Drivetrain motors
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private final HardwareMap hwMap;
    public Gyro imu;
    public Odometry odometry;
    public ShooterSubsystem shooter;
    public Turret turret;
    public HiveState hiveState;
    public FlowerRamp flowerRamp;

    // Every subsystem, in the order update() runs them. Odometry first so later steps see a fresh pose.
    private List<Subsystem> subsystems;

    public RobotHardware(HardwareMap hardwareMap, HiveState.Alliance alliance) {
        this.hwMap = hardwareMap;
        init(alliance);
    }

    private void init(HiveState.Alliance alliance) {
        // --- Mechanisms ---
        imu = new Gyro(hwMap);
        odometry = new Odometry(hwMap);
        shooter = new ShooterSubsystem(hwMap);
        turret = new Turret(hwMap);
        hiveState = new HiveState(alliance);
        flowerRamp = new FlowerRamp(hwMap);
        subsystems = Arrays.asList(odometry, imu, hiveState, turret, shooter, flowerRamp);

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

    /** Call once per loop: reads the Pinpoint and runs each subsystem's control. */
    public void update() {
        for (Subsystem s : subsystems) s.update();
    }

    /** Call when the OpMode ends. */
    public void stop() {
        for (Subsystem s : subsystems) s.stop();
        driveMecanum(0, 0, 0);
    }

    /** Adds every subsystem's lines. The OpMode still calls telemetry.update() once afterwards. */
    public void addTelemetry(Telemetry telemetry) {
        for (Subsystem s : subsystems) s.addTelemetry(telemetry);
    }

    /**
     * Points the turret at the up cell of our hive and sets flywheel speed and hood angle for the distance.
     * @return the shot ShotSolver computed, for telemetry or to check before feeding
     */
    public ShotSolver.Shot aimAtHive() {
        ShotSolver.Shot shot = ShotSolver.solve(odometry.getX(), odometry.getY(), odometry.getHeadingDeg(),
                hiveState.getTargetPosition());
        turret.setTargetAngle(shot.turretAngleDeg);
        shooter.setTargetVelocity(shot.rpm);
        shooter.setHoodAngle(shot.hoodDeg);
        return shot;
    }

    /** @return true when the turret is on target and the flywheel is at speed, i.e. safe to feed a ball */
    public boolean readyToShoot() {
        return turret.atTarget() && shooter.atTargetVelocity() && shooter.getTargetVelocity() > 0;
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
