package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    public Turret() { }

    private ServoEx servo;

    @Override
    public void initialize() {
        servo = new ServoEx("turretPivot");
    }
}
