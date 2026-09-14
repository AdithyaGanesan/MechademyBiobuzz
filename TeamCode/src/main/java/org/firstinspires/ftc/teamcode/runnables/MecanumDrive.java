package org.firstinspires.ftc.teamcode.runnables;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.RobotHardware;

/**
 * General Driving OpMode
 * @author: AdithyaGanesan
 */
@TeleOp(name = "Mecanum TeleOp")
public class MecanumDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double axial   = -gamepad1.left_stick_y; // Y is inverted on gamepad
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            robot.driveMecanum(axial, lateral, yaw);

            telemetry.addData("Axial", axial);
            telemetry.addData("Lateral", lateral);
            telemetry.addData("Yaw", yaw);
            telemetry.update();
        }
    }
}