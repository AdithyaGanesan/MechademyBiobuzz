package org.firstinspires.ftc.teamcode.runnables;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.HiveState;
import org.firstinspires.ftc.teamcode.mechanisms.RobotHardware;
import org.firstinspires.ftc.teamcode.mechanisms.ShotSolver;

/**
 * Match TeleOp shared by TeleOpRed and TeleOpBlue; only the alliance differs.
 * Doesn't zero the turret or reset the Pinpoint, so both carry over from auton.
 *
 * gamepad1: left stick = drive, right stick X = turn
 * gamepad2: A = aiming on/off, B = re-zero turret (point it forward first),
 *           dpad up = hive tipped (flip target cell), X = flower ramp up/down
 * @author: AdithyaGanesan
 */
public abstract class BiobuzzTeleOp extends LinearOpMode {

    protected abstract HiveState.Alliance alliance();

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap, alliance());
        boolean aiming = false;

        telemetry.addData("Alliance", alliance());
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            robot.update();

            // --- Driver 1: drive ---
            double axial   = -gamepad1.left_stick_y; // Y is inverted on gamepad
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            robot.driveMecanum(axial, lateral, yaw);

            // --- Driver 2: mechanisms ---
            if (gamepad2.aWasPressed()) aiming = !aiming;
            if (gamepad2.bWasPressed()) robot.turret.zeroHere();
            if (gamepad2.dpadUpWasPressed()) robot.hiveState.flip();
            if (gamepad2.xWasPressed()) robot.flowerRamp.toggle();

            if (aiming) {
                ShotSolver.Shot shot = robot.aimAtHive();
                telemetry.addData("Shot distance (in)", "%.1f", shot.distanceIn);
            } else {
                robot.turret.setTargetAngle(0);
                robot.shooter.stop();
            }
            //TODO: Feed balls with the transfer once it exists, only while robot.readyToShoot().

            telemetry.addData("Aiming", aiming);
            telemetry.addData("Ready to shoot", robot.readyToShoot());
            robot.addTelemetry(telemetry);
            telemetry.update();
        }

        robot.stop();
    }
}
