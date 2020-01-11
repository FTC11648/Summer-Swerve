package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//This opmode will drag the foundation into the build zone if you are on the BLUE team

@Autonomous(name="Foundation(RED)", group="Pushbot")

public class FoundationRED extends LinearOpMode {
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

        robotMover.clampLeft.setPosition(0.6);
        robotMover.clampRight.setPosition(0.6);
        sleep(500);

        //Do the course
        robotMover.encoderDrive(0.6, 31.5, 31.5, 0);

        //Grab foundation
        robotMover.clampLeft.setPosition(-0.8);
        robotMover.clampRight.setPosition(-0.8);
        sleep(500);

        robotMover.encoderDrive(0.6, -31.5, -31.5, 0);

        robotMover.clampLeft.setPosition(0.6);
        robotMover.clampRight.setPosition(0.6);

        robotMover.encoderDrive(0.6, 0, 0, -50);
    }
}