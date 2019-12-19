package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


public class DriftDrive implements Subsystem {

    Gamepad gamepad1;
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor centerDrive;
    /* Declare OpMode members. */

    public DriftDrive(Gamepad gamepad1, DcMotor leftDrive,DcMotor rightDrive,DcMotor centerDrive)
    {
        this.gamepad1 = gamepad1;

        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        this.centerDrive = centerDrive;

    }


    @Override
    public void init() {


    }

    @Override
    public void update() {
        double left;
        double right;
        double center;
        double driveVertical;
        double driveHorizontal;
        double turn;
        double max;

        driveVertical = -gamepad1.left_stick_y;
        driveHorizontal = gamepad1.left_stick_x;
        turn  =  gamepad1.right_stick_x;

        // Combine drive and turn for blended motion.
        left  = driveVertical + turn;
        right = driveVertical - turn;
        center = driveHorizontal;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Output the safe vales to the motor drives.
        leftDrive.setPower(left);
        rightDrive.setPower(right);
        centerDrive.setPower(center);
    }
}
