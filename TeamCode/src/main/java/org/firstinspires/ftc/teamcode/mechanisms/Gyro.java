package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Accesses IMU on Control Hub.
 * @author: AdithyqGanesan
 **/
public class Gyro implements Subsystem {
    // Control Hub is on the right side wall: logo faces right and tilts down by the wall's slant,
    // USB ports face down. Tilt is from the CAD. If pitch/roll don't read ~0 with the robot flat, flip the sign.
    private static final double HUB_TILT_DEG = 22;

    private final IMU imu;

    public Gyro(HardwareMap hardwareMap){
        imu = hardwareMap.get(IMU.class,"imu");

        // With HUB_TILT_DEG = 0 this is the same as LogoFacingDirection.RIGHT + UsbFacingDirection.DOWN.
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.xyzOrientation(180, 90 - HUB_TILT_DEG, 90));

        imu.initialize(new IMU.Parameters(RevOrientation));
    }
    public double getHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    @Override
    public void update() {}

    /** Pitch and roll should read ~0 with the robot flat; if not, the hub tilt setting is wrong. */
    @Override
    public void addTelemetry(Telemetry telemetry) {
        YawPitchRollAngles a = imu.getRobotYawPitchRollAngles();
        telemetry.addData("IMU heading", "%.1f°", a.getYaw(AngleUnit.DEGREES));
        telemetry.addData("IMU pitch / roll", "%.1f° / %.1f°", a.getPitch(AngleUnit.DEGREES), a.getRoll(AngleUnit.DEGREES));
    }
}
