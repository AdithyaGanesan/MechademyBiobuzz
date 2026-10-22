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
    private final IMU imu;

    public Gyro(HardwareMap hardwareMap){
        imu = hardwareMap.get(IMU.class,"imu");

        //TODO: Calibrate IMU orientation. CAD has the hubs on the side walls tilted ~22°;
        // still need which wall the Control Hub is on and which way its USB ports face.
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }
    public double getHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
