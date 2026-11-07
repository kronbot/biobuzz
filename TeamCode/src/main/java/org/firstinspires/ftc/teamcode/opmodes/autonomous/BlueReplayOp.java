package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.Constants;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Replay - Blue Alliance", group = Constants.MAIN_GROUP,
        preselectTeleop = "Driving - Blue Alliance")
public final class BlueReplayOp extends ReplayOp {
    public BlueReplayOp(KronBot robot) {
        super(robot, Alliance.BLUE);
    }
}
