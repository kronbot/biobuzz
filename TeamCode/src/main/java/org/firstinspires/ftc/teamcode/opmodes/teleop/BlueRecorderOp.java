package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.RecordingSession;

import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "Recorder - Blue Alliance", group = Constants.MAIN_GROUP)
public final class BlueRecorderOp extends BlueAllianceTeleOp {
    public BlueRecorderOp(KronBot robot) {
        super(robot, new RecordingSession(robot, Alliance.BLUE));
    }
}
