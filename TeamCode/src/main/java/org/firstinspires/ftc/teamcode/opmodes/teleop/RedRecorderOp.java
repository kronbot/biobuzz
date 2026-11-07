package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.utils.Constants;

import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "Recorder - Red Alliance", group = Constants.MAIN_GROUP)
public final class RedRecorderOp extends RecorderOp {
    public RedRecorderOp(KronBot robot) {
        super(robot, false);
    }
}
