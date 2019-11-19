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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.math.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@TeleOp(name="StarTrek: GTA Driving", group="Pushbot")
public class GTADriving extends OpMode{
    int breakVal = 0;
    boolean loopFinished = false;

    /* Declare OpMode members. */
    HardwareStarTrek robot = new HardwareStarTrek(); // use the class created to define a Pushbot's hardware

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Ben, Hopefully you can drive.");    //
    }


    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    //This is the actual driving controls
    @Override
    public void loop() {

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        float forward = gamepad1.left_trigger; //Max:1
        float breaks = gamepad1.right_trigger; //Max:1
        float turn = gamepad1.right_stick_x; //Max:1
        telemetry.addData("Say", Float.toString(forward + breaks));

        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.backRight.setDirection(DcMotor.Direction.REVERSE);
          /*  if(Math.abs(strafe) > 0) {
                robot.frontLeft.setPower(-strafe);
                robot.backLeft.setPower(strafe);
                robot.frontRight.setPower(-strafe);
                robot.backRight.setPower(strafe);
            }
           */
        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;
        final double v0 = r * Math.cos(robotAngle) + rightX;
        final double v1 = r * Math.sin(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.cos(robotAngle) - rightX;
        //if(!isButtonDown(gamepad1.left_trigger) || !isButtonDown(gamepad1.right_trigger)) {
        robot.frontLeft.setPower(v0);
        robot.frontRight.setPower(v1);
        robot.backLeft.setPower(v2);
        robot.backRight.setPower(v3);
        //}

        if (gamepad1.left_bumper) {
            robot.FrontClaw.setPosition(robot.FrontClaw.getPosition()+0.02);
            telemetry.addData("Front Claw", robot.FrontClaw.getPosition());
        }

        if (gamepad1.right_bumper) {
            robot.Hook.setPosition(robot.Hook.getPosition()+0.02);
            telemetry.addData("Hook", robot.FrontClaw.getPosition());
        }
/*


            if(Math.abs(turn) > 0) {
                if(isButtonDown(forward)) {
                    robot.frontLeft.setPower(-(turn-0.8));
                    robot.backLeft.setPower(-(turn-0.8)); //If doesn't work write -(turn-0.8)
                    robot.backRight.setPower(-(turn));
                    robot.frontRight.setPower(-(turn));
                } else {
                    robot.frontLeft.setPower(-(turn));
                    robot.backLeft.setPower(-(turn)); //If doesn't work write -(turn-0.8)
                    robot.backRight.setPower(-(turn));
                    robot.frontRight.setPower(-(turn));
                }
            }
            */
// trip sucks ey
// How hysterical. I laughed.

        /*if(!isButtonDown(gamepad1.left_stick_x) || !isButtonDown(gamepad1.left_stick_y))
            robot.backLeft.setPower(forward-breaks); //Max: 1
            robot.backRight.setPower(-forward+breaks); //Max: 1
            robot.frontLeft.setPower(forward-breaks); //Max: 1
            robot.frontRight.setPower(-forward+breaks); //Max: 1
         */

    }


    public boolean isButtonDown(float gamePad) {
        if(gamePad==0) return false;
        else return true;
    }
    // Use gamepad left & right Bumpers to open and close the claw
    //Ty FTC but I think Im good 4 now ;)
        /*if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            clawOffset -= CLAW_SPEED;
                */

        /*
        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
        robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad1.y)
            robot.leftArm.setPower(robot.ARM_UP_POWER);
        else if (gamepad1.a)
            robot.leftArm.setPower(robot.ARM_DOWN_POWER);
        else
            robot.leftArm.setPower(0.0);

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
    }
    */
    /*
     * Code to run ONCE after the driver hits STOP
     */
}


