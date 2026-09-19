package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.KronBot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "Simple Driving TeleOp", group = "Teleop")
public class SimpleDrivingTeleOp extends NextOpMode {
    private final KronBot robot;

    public SimpleDrivingTeleOp(KronBot robot) {
        super(robot);
        this.robot = robot;
    }

    @Override
    public void start() {
        robot.drivetrain.startDrive(gamepad1);
    }
}