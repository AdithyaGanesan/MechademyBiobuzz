package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Pedro Pathing setup: mecanum drivetrain, two odometry pods + hub IMU, and the Foresight follower.
 * Most values come from the Pedro tuners, which need the built robot. Until those are pasted in,
 * create() stops with "Config variable has not been set" instead of handing back a null Follower.
 */
public class Constants {
    // Names match RobotHardware. Directions copy RobotHardware for now; confirm with the Mecanum Tuner.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    //TODO: Paste the Two Wheel Tuner output here (pod names, offsets, ticks-to-inches, directions, IMU orientation).
    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(c -> {
    });

    //TODO: Paste the Foresight Tuner output here.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
    });

    public static Drivetrain createDrivetrain(HardwareMap h) {
        return new Mecanum(h, drivetrainConfig);
    }

    public static Localizer createLocalizer(HardwareMap h) {
        return new TwoWheelLocalizer(h, localizerConfig);
    }

    public static Follower create(HardwareMap h) {
        return new Follower(createLocalizer(h), createDrivetrain(h), new Foresight(foresightConfig));
    }
}
