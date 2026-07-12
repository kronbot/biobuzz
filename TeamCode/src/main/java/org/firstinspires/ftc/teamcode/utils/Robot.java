package org.firstinspires.ftc.teamcode.utils;

import android.telecom.TelecomManager;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.UtilityOctoQuadConfigMenu;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Robot {
    // Fundamental
    private static Robot instance = null;
    private static HardwareMap hardwareMap;
    public List<LynxModule> allHubs;
    public TelemetryManager telemetryManager;

    // Software subsystems
    public Follower follower;

    // Hardware parts
    public DcMotorEx leftRear, leftFront, rightRear, rightFront;

    // Other
    public static Pose startingPose = null;

    public static Robot getInstance() {
        if(instance == null) instance = new Robot();
        return instance;
    }

    /**
     * Get all references to the hardware components from the HardwareMap
     * @param hardwareMap THE HardwareMap
     */
    public void initHardware(HardwareMap hardwareMap) {
        Robot.hardwareMap = hardwareMap;

        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        ArrayList<DcMotorEx> motors = new ArrayList<DcMotorEx>(Arrays.asList(leftRear, rightRear, leftFront, rightFront));

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }


    }

    /**
     * Initialize all the subsystems
     *  <br>
     * Set robot.startingPose before calling this
     */
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        // ex:
        // if(drivetrain != null) drivetrain.init();
    }

    /*
     * Possibly add initHardwareAuto() and initAuto() if needed?
     */

    /**
     * MUST be called frequently (from the non-blocking cooperative loop)!<br>
     * Otherwise the robot will go "haywire"!
     */
    public void loop() {
        for(LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}
