package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.RecordingSession;

import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "Driving - Blue Alliance", group = Constants.MAIN_GROUP)
public class BlueAllianceTeleOp extends AllianceDrivingTeleOp {
    public BlueAllianceTeleOp(KronBot robot) {
        this(robot, null);
    }

    protected BlueAllianceTeleOp(KronBot robot, RecordingSession recording) {
        super(robot, Alliance.BLUE, recording);
    }
}
