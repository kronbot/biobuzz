package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.RecordingSession;

import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "Recorder - Red Alliance", group = Constants.MAIN_GROUP)
public final class RedRecorderOp extends RedAllianceTeleOp {
    public RedRecorderOp(KronBot robot) {
        super(robot, new RecordingSession(robot, Alliance.RED));
    }
}
