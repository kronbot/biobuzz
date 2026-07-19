package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Constants.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flap;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Loader;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import dev.nextftc.bindings.Range;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

@TeleOp(name = "MainTeleOp", group = "Test")
public class MainTeleOp extends NextFTCOpMode {
    public MainTeleOp() {
        addComponents(
                new SubsystemComponent(
                        Outtake.INSTANCE,
                        Intake.INSTANCE,
                        Loader.INSTANCE,
                        Flap.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                true // robot-centric (false for field centric)
        );
        driverControlled.schedule();

        double driverLoaderSpeed =
                Gamepads.gamepad1().rightTrigger().deadZone(0.1).get() -
                Gamepads.gamepad1().leftTrigger().deadZone(0.1).get();
        double utilityLoaderSpeed =
                Gamepads.gamepad2().rightStickY().deadZone(0.1).get();

        Gamepads.gamepad1().rightBumper()
                .whenFalse(() -> Flap.INSTANCE.close()
                        .and(Loader.INSTANCE.activate(utilityLoaderSpeed))
                        .and(Intake.INSTANCE.activate(utilityLoaderSpeed))
                        .invoke())
                .whenTrue(() -> Flap.INSTANCE.open()
                        .and(Loader.INSTANCE.activate(driverLoaderSpeed))
                        .and(Intake.INSTANCE.activate(driverLoaderSpeed / 2))
                        .invoke()
                );

        Gamepads.gamepad1().triangle().whenBecomesTrue(
                () -> Outtake.INSTANCE.setGoalVelocity(RANGE_1_VELOCITY)
                        .then(new Command() {
                            @Override
                            public void start() {
                                gamepad1.rumble(1, 1, 100);
                            }
                            @Override
                            public boolean isDone() {
                                return !gamepad1.isRumbling();
                            }
                        }.requires(gamepad1))
                        .invoke()
        );
    }

}
