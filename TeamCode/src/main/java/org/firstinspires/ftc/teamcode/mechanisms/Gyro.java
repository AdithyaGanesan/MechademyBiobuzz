package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Accesses IMU on Control Hub.
 * @author: AdithyqGanesan
 **/
public class Gyro {
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
}
