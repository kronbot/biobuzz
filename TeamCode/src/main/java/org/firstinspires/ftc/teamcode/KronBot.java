package org.firstinspires.ftc.teamcode;

import java.util.Collections;
import java.util.Set;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;

public class KronBot implements NextRobot {
    public final NextMotor frontLeft;
    public final NextMotor backLeft;
    public final NextMotor frontRight;
    public final NextMotor backRight;
    public final Drivetrain drivetrain;

    public KronBot() {
        frontLeft = new NextMotor(RobotController.controlHub(),2);
        backLeft = new NextMotor(RobotController.controlHub(),1);
        frontRight = new NextMotor(RobotController.controlHub(),3);
        backRight = new NextMotor(RobotController.controlHub(),0);

        frontRight.setDirection(NextMotor.Direction.REVERSE);
        backRight.setDirection(NextMotor.Direction.REVERSE);

        drivetrain = new Drivetrain(frontLeft, frontRight, backLeft, backRight);
    }

    @Override
    public void periodic() {
    }

    @Override
    public Set<Mechanism> getMechanisms() {
        return Collections.emptySet();
    }
}