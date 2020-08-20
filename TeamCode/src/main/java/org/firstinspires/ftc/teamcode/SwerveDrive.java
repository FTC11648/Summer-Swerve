package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class SwerveDrive implements Subsystem {

    private DcMotor angleMotor;
    private DcMotor speedMotor;

    Gamepad gamePad1;
    DcMotor frontLeftSpeed;
    DcMotor backLeftSpeed;
    DcMotor frontRightSpeed;
    DcMotor backRightSpeed;

    Servo frontLeftAngle;
    Servo backLeftAngle;
    Servo frontRightAngle;
    Servo backRightAngle;

    public final double L = 10 ;
    public final double W = 7.75;

    /* Declare OpMode members. */

    public SwerveDrive(Gamepad gamePad1, DcMotor frontRightSpeed, DcMotor backRightSpeed,
                       DcMotor frontLeftSpeed, DcMotor backLeftSpeed,  Servo frontLeftAngle,
                       Servo backLeftAngle, Servo frontRightAngle, Servo backRightAngle) {
        this.gamePad1 = gamePad1;

        this.frontLeftSpeed = frontLeftSpeed;
        this.backLeftSpeed = backLeftSpeed;
        this.frontRightSpeed = frontRightSpeed;
        this.backRightSpeed = backRightSpeed;

        this.frontLeftAngle = frontLeftAngle;
        this.backLeftAngle = backLeftAngle;
        this.frontRightAngle = frontRightAngle;
        this.backRightAngle = backRightAngle;
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

        driveVertical = -gamePad1.left_stick_y;
        driveHorizontal = gamePad1.left_stick_x;
        turn = gamePad1.right_stick_x;

        double r = Math.sqrt ((L * L) + (W * W));

        double a = gamePad1.left_stick_x - gamePad1.right_stick_x * (L / r);
        double b = gamePad1.left_stick_x + gamePad1.right_stick_x * (L / r);
        double c = -gamePad1.left_stick_y - gamePad1.right_stick_x * (W / r);
        double d = -gamePad1.left_stick_y + gamePad1.right_stick_x * (W / r);

        backRightSpeed.setPower(Math.sqrt ((a * a) + (d * d)));
        backLeftSpeed.setPower(Math.sqrt ((a * a) + (c * c)));
        frontRightSpeed.setPower(Math.sqrt ((b * b) + (d * d)));
        frontLeftSpeed.setPower(Math.sqrt ((b * b) + (c * c)));

        backRightAngle.setPosition(Math.atan2 (a, d) / Math.PI / 360);
        backLeftAngle.setPosition(Math.atan2 (a, c) / Math.PI / 360);
        frontRightAngle.setPosition(Math.atan2 (b, d) / Math.PI / 360);
        frontLeftAngle.setPosition(Math.atan2 (b, c) / Math.PI / 360);
    }
}
