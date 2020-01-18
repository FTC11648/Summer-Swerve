package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//This opmode will drag the foundation into the build zone if you are on the BLUE team

@Autonomous(name="ParkBLUE", group="Pushbot")

public class Straight extends LinearOpMode {
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


        robotMover.grab(robot.RELEASE_POSITION);
        sleep(500);



        robotMover.encoderDrive(0.6, 140, 140, 0);


    }
}