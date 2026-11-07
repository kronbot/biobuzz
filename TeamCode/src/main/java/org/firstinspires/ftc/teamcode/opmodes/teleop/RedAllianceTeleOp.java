package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.RecordingSession;

import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "Driving - Red Alliance", group = Constants.MAIN_GROUP)
public class RedAllianceTeleOp extends AllianceDrivingTeleOp {
    public RedAllianceTeleOp(KronBot robot) {
        this(robot, null);
    }

    protected RedAllianceTeleOp(KronBot robot, RecordingSession recording) {
        super(robot, Alliance.RED, recording);
    }
}
