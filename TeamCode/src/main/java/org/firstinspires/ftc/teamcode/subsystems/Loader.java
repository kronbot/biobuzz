package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Loader implements Subsystem {
    public static final Loader INSTANCE = new Loader();
    private Loader() { }

    private CRServoEx crServo;

    public Command activate(double speed) {
        return new SetPower(crServo, speed).requires(this);
    }

    public Command deactivate() {
        return new SetPower(crServo, 0).requires(this);
    }

    @Override
    public void initialize() {
        crServo = new CRServoEx("loader");
    }

}
