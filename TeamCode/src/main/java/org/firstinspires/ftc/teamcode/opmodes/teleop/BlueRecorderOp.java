package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.utils.Constants;

import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "Recorder - Blue Alliance", group = Constants.MAIN_GROUP)
public final class BlueRecorderOp extends RecorderOp {
    public BlueRecorderOp(KronBot robot) {
        super(robot, true);
    }
}
