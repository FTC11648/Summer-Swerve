package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//This opmode will drag the foundation into the build zone if you are on the BLUE team

@Autonomous
public class BlockToBridgeBLUE extends LinearOpMode {
    Hardware robot;
    RobotMover robotMover;

    @Override
    public void runOpMode() {
        robot = new Hardware(hardwareMap);
        robot.initDriveTrain();
        robot.initFourBar();
        robot.initBlinkinAuto(0);

        robotMover = new RobotMover(robot.leftDrive, robot.rightDrive, robot.centerDrive, robot.imu, robot.leftArm,robot.rightArm, robot.clampRight, robot.clampLeft);

        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.centerDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status: ", "Waiting for start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //release for start
        robotMover.grab(robot.RELEASE_POSITION);
        sleep(500);

        //Go forward
        robotMover.encoderDrive(0.6, 31.5, 31.5, 0,3);

        //Grab block
        robotMover.grab(robot.GRAB_POSITION);
        sleep(500);

        //Move arm up
        robotMover.arm(robot.ARM_UP);
        sleep(500);

        //Back up 5 inches
        robotMover.encoderDrive(0.6,-5,-5,0,3);

        //Turn left 90 degrees
        robotMover.rotate(90);

        //Go forward and park under skybridge
        robotMover.encoderDrive(0.6, 55, 55, 0,4);

    }
}
