package org.firstinspires.ftc.teamcode.runnables;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.HiveState;

/**
 * Match TeleOp for the red alliance.
 * @author: AdithyaGanesan
 */
@TeleOp(name = "TeleOp Red", group = "Competition")
public class TeleOpRed extends BiobuzzTeleOp {
    @Override
    protected HiveState.Alliance alliance() {return HiveState.Alliance.RED;}
}
