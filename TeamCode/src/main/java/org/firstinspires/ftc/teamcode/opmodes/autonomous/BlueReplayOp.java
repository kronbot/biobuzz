package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.Constants;
import org.psilynx.psikit.ftc.autolog.PsiKitNoAutoLog;

@PsiKitNoAutoLog
@Autonomous(name = "Replay - Blue Alliance", group = Constants.MAIN_GROUP)
public final class BlueReplayOp extends ReplayOp {
    public BlueReplayOp() {
        super(true);
    }
}
