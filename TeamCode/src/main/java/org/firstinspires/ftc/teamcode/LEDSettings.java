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
    ElapsedTime timer = new ElapsedTime();

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
        int team = 0; //1 is red, 0 is blue

        if (gamepad1.start) { //blue teleop
            Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
            team = 0;
        }
        else if (gamepad1.back) { //red teleop
            Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
            team = 1;


        }
        if (timer.seconds() > 90 && team == 0) {
            Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW);
        }
         if (timer.seconds() > 115 && team == 0) {
            Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_MEDIUM);
        }
         if (timer.seconds() > 125 && team == 0) {
            Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
        }
         if (timer.seconds() > 90 && team == 1) {
            Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_SLOW);
        }
         if (timer.seconds() > 115 && team == 1) {
            Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_MEDIUM);
        }
         if (timer.seconds() > 125 && team == 1) {
            Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST);
        }

/*
*/
    }
}