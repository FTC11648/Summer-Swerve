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

        robotMover.grab(robot.RELEASE_POSITION);
        sleep(500);

        //Do the course
        robotMover.encoderDrive(0.6, 31.5, 31.5, 0);
        robotMover.grab(robot.GRAB_POSITION);
        sleep(500);

        //Grab foundation
        robotMover.rotate(104.15);
        robotMover.encoderDrive(0.6, 55, 55, 0);
        robotMover.rotate(-14.15);
    }
}
