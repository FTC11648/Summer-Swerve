package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.lang.Object;

import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

import static java.lang.Thread.sleep;

public class LEDSettings implements Subsystem {
    private boolean FInput=true;
    /*
     * Change the pattern every 10 seconds in AUTO mode.
     */
    private final static int LED_PERIOD = 100;
    ElapsedTime runtime = new ElapsedTime();


    private final static int GAMEPAD_LOCKOUT = 500;


    HardwareMap Hwp = null;
    Telemetry.Item patternName;
    Telemetry.Item display;
    Gamepad gamepad1;
    RevBlinkinLedDriver Lights;


    public LEDSettings(Gamepad gamepad1, RevBlinkinLedDriver Lights){
        this.gamepad1=gamepad1;
        this.Hwp = Hwp;
        this.Lights = Lights;
        init();
    }


    @Override
    public void init() {

    }

    @Override
    public void update() {

/*
*/
    }
}