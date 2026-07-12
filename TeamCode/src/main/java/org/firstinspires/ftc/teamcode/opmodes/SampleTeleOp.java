package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

/**
 * A sample TeleOp to be copied when creating a new TeleOp.
 * The sample TeleOp should be edited to keep it up to date with the rest of the code.
 * <br>B)<br>
 * Right now it has basic PedroPathing drive
 */

@TeleOp(name = "SampleTeleOp", group = "Test")
public class SampleTeleOp extends NextFTCOpMode {
    public SampleTeleOp() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY(),
            Gamepads.gamepad1().leftStickX(),
            Gamepads.gamepad1().rightStickX(),
            true // robot-centric (false for field centric)
        );
        driverControlled.schedule();
    }
}
