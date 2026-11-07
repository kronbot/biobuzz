package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.Constants;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Replay - Red Alliance", group = Constants.MAIN_GROUP,
        preselectTeleop = "Driving - Red Alliance")
public final class RedReplayOp extends ReplayOp {
    public RedReplayOp(KronBot robot) {
        super(robot, Alliance.RED);
    }
}
