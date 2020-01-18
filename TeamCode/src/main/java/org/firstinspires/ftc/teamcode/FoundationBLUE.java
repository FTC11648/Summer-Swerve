package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//This opmode will drag the foundation into the build zone if you are on the BLUE team

@Autonomous(name="Foundation(BLUE)", group="Pushbot")

public class FoundationBLUE extends LinearOpMode {
    Hardware robot;
    RobotMover robotMover;

    @Override
    public void runOpMode() {
        robot = new Hardware(hardwareMap);
        robot.initDriveTrain();
        robot.initFourBar();
        robot.initBlinkinAuto(1);

        robotMover = new RobotMover(robot.leftDrive, robot.rightDrive, robot.centerDrive, robot.imu, robot.leftArm,robot.rightArm, robot.clampRight, robot.clampLeft);

        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.centerDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status: ", "Waiting for start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Put arm down to get foundation
        robotMover.arm(0.245);

        //Release for start
        robotMover.grab(robot.RELEASE_POSITION);
        sleep(500);

        //Go left 18 inches
        robotMover.encoderDrive(0.8,0,0,-18,3);

        //Go forward to foundation
        robotMover.encoderDrive(0.8, 31.5, 31.5, 0,3);

        //Grab foundation
        robotMover.grab(robot.GRAB_POSITION);
        sleep(500);

        //Back up with foundation
        robotMover.encoderDrive(0.9, -33.5, -33.5, 0,3);

        //Release foundation and move up arm
        robotMover.grab(robot.RELEASE_POSITION);
        robotMover.arm(0.257);

        //Back into wall
        robotMover.encoderDrive(0.4,-10,-10,0,3);
        robotMover.encoderDrive(0.8,2,2,0,3);

        //Park under skybridge
        robotMover.encoderDrive(0.8, 0, 0, 85,5);

    }
}