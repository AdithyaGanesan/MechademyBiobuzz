package org.firstinspires.ftc.teamcode.runnables;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.HiveState;

/**
 * Match TeleOp for the blue alliance.
 * @author: AdithyaGanesan
 */
@TeleOp(name = "TeleOp Blue", group = "Competition")
public class TeleOpBlue extends BiobuzzTeleOp {
    @Override
    protected HiveState.Alliance alliance() {return HiveState.Alliance.BLUE;}
}
