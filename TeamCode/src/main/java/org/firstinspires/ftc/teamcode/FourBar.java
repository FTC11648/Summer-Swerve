/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


public class FourBar implements Subsystem {


    Gamepad gamepad2;
    Servo leftArm;
    Servo rightArm;
    Servo clampLeft;
    Servo clampRight;
    /* Declare OpMode members. */
    public double   armOffset      = 0.17;                       // Servo mid position
    public double   clampOffset = 0;                      // Claw mid position
    final double    ARM_SPEED      = 0.0006 ;                   // sets rate to move
    final double    CLAMP_SPEED      = 0.005;                    // sets rate to move

    public FourBar(Gamepad gamepad2, Servo leftArm, Servo rightArm, Servo clampLeft, Servo clampRight)
    {
        this.gamepad2 = gamepad2;
        this.leftArm = leftArm;
        this.rightArm = rightArm;
        this.clampLeft = clampLeft;
        this.clampRight = clampRight;


    }
    @Override
    public void init() {

    }

    @Override
        public void update  () {
                armOffset = leftArm.getPosition();

            // Four bar movement
            if (gamepad2.right_bumper) {
                armOffset = armOffset + ARM_SPEED;
                armOffset = Range.clip(armOffset, 0.243, 0.76);
                leftArm.setPosition(armOffset);
                rightArm.setPosition(armOffset);
            }
            else if (gamepad2.left_bumper) {
                armOffset = armOffset - ARM_SPEED;
                armOffset = Range.clip(armOffset, 0.243, 0.76);
                leftArm.setPosition(armOffset);
                rightArm.setPosition(armOffset);
            }
            else if (gamepad2.a) {
                armOffset = 0.243;
                leftArm.setPosition(armOffset);
                rightArm.setPosition(armOffset);
            }
            else if (gamepad2.b) {
                armOffset = 0.33;
                leftArm.setPosition(armOffset);
                rightArm.setPosition(armOffset);
            }
            else if (gamepad2.y) {
                armOffset = 0.375;
                leftArm.setPosition(armOffset);
                rightArm.setPosition(armOffset);
            }
            else if (gamepad2.x) {
                armOffset = 0.42;
                leftArm.setPosition(armOffset);
                rightArm.setPosition(armOffset);
            }
            else if (gamepad2.back) {
                armOffset = 0.48;
                leftArm.setPosition(armOffset);
                rightArm.setPosition(armOffset);
            }





            //Clamp movement
            else if(gamepad2.dpad_right) {
                clampOffset = clampOffset + CLAMP_SPEED;
                clampOffset = Range.clip(clampOffset, -0.8, 0.6 );
                clampLeft.setPosition(0 + clampOffset);
                clampRight.setPosition(0 + clampOffset);
            }
            else if(gamepad2.dpad_left) {
                clampOffset = clampOffset - CLAMP_SPEED;
                clampOffset = Range.clip(clampOffset, -0.8, 0.6);
                clampLeft.setPosition(0 + clampOffset);
                clampRight.setPosition(0 + clampOffset);
            }
            else if (gamepad2.dpad_up) {
                clampOffset = 0.6;
                clampLeft.setPosition(clampOffset);
                clampRight.setPosition(clampOffset);
            }
            else if (gamepad2.dpad_down) {
                clampOffset = -0.8;
                clampLeft.setPosition(clampOffset);
                clampRight.setPosition(clampOffset);
            }

        }
}
