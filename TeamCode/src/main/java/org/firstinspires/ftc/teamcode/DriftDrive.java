package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


public class DriftDrive implements Subsystem {

    Gamepad gamepad1;
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor centerDrive;
    RobotMover robotMover;
    /* Declare OpMode members. */


    public DriftDrive(Gamepad gamepad1, DcMotor leftDrive, DcMotor rightDrive, DcMotor centerDrive, BNO055IMU imu, Servo leftArm, Servo rightArm, Servo clampRight, Servo clampLeft)
    {
        this.gamepad1 = gamepad1;

        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        this.centerDrive = centerDrive;

        robotMover = new RobotMover(leftDrive, rightDrive, centerDrive, imu, leftArm, rightArm, clampRight, clampLeft);

    }


    @Override
    public void init() {


    }

    @Override
    public void update() {
        //I didn't want to initialize them on one line, fight me
        double left;
        double right;
        double center;
        double driveVertical;
        double driveHorizontal;
        double turn;
        double y;
        double x;
        double power;
        double driveHeading;


        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;

        //LOTS OF MATH BEGINS
        power = Math.sqrt((x*x)+(y*y));

        driveHeading = Math.atan2(y, x)-(Math.PI/2);
        turn = gamepad1.right_stick_x;

        driveVertical = Math.cos(Math.toRadians(robotMover.getAngle()-Math.toDegrees(driveHeading)));
        driveHorizontal = Math.sin(Math.toRadians(robotMover.getAngle()-Math.toDegrees(driveHeading)));

        //There are two motors on the side, so double center to compensate
        left = driveVertical*power + turn;
        right = driveVertical*power - turn;
        center = driveHorizontal*power*2;

        //Normalize
        if(Math.max(Math.max(left, right), center) > 1) {
            double scale = 1/Math.max(Math.max(left, right), center);
            left = left*scale;
            right = right*scale;
            center = center*scale;
        }

        //Run motors
        leftDrive.setPower(left);
        rightDrive.setPower(right);
        centerDrive.setPower(center);
    }
}
