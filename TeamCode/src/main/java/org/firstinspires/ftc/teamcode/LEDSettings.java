package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

public class LEDSettings implements Subsystem {
    private boolean FInput=true;
    /*
     * Change the pattern every 10 seconds in AUTO mode.
     */
    private final static int LED_PERIOD = 100;
    ElapsedTime runtime = new ElapsedTime();
    boolean istrue=false;
    /*
     * Rate limit gamepad button presses to every 500ms.
     */
    private final static int GAMEPAD_LOCKOUT = 500;


    HardwareMap Hwp = null;
    Telemetry.Item patternName;
    Telemetry.Item display;
    FirstLEDSettings.DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;
    Gamepad gamepad1;
    RevBlinkinLedDriver Lights;
    protected enum DisplayKind {
        MANUAL,
        AUTO
    }

    public LEDSettings(Gamepad gamepad1, RevBlinkinLedDriver Lights, HardwareMap Hwp){
        this.gamepad1=gamepad1;
        this.Hwp=Hwp;
        this.Lights = Lights;
        init();
    }


    @Override
    public void init() {
        /*displayKind = FirstLEDSettings.DisplayKind.MANUAL; //This is used to change between the Manual and the Auto mode
        //blinkinLedDriver = Hwp.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;           //Here is where you can change the color of the Lights
        Lights.setPattern(pattern);
        handleGamepad();*/
        Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
    }
    public void handleGamepad() {

        /*if (gamepad1.a && FInput) {     //This should check for input from the Gamepad and if it is detected then it will switch the colors while starting the Timer!
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;
            update();
           // handleTimer();
            this.FInput=false;u
            // gamepadRateLimit.reset();
        } else if (gamepad1.b && FInput) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;
            update();
           // handleTimer();
            this.FInput=false;
            //gamepadRateLimit.reset();
        } else if (gamepad1.left_bumper && FInput) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;
            update();
           // handleTimer();
            this.FInput=false;
            //gamepadRateLimit.reset();
        } else if (gamepad1.right_bumper && FInput) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;
            update();
            //handleTimer();
            this.FInput=false;
            //gamepadRateLimit.reset();
        }else if (gamepad1.x && FInput){
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;
            update();
            //handleTimer();
            this.FInput=false;
            //gamepadRateLimit.reset();
        }else if (gamepad1.y && FInput){
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;
            update();
            //handleTimer();
            this.FInput=false;
            //  gamepadRateLimit.reset();
        }*/
        if (gamepad1.a)
        {
            Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
        }



    }



    @Override
    public void update() {

        /*Lights.setPattern(pattern);
        patternName.setValue(pattern.toString());*/
/*
        int interval = 10000; // This is in milli seconds! (10 seconds)pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;
        Date timeToRun = new Date(System.currentTimeMillis() + interval);
        Timer timer = new Timer();
        timer.schedule(new TimerTask() {
            public void run() {
                pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
                blinkinLedDriver.setPattern(pattern);                  //This is executed after the time has expired!
            }
        }, timeToRun);
*/
    }
}