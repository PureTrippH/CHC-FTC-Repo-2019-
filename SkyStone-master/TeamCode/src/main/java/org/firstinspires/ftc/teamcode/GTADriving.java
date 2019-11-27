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

//Imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//Main
@TeleOp(name="StarTrek: GTA Driving")
public class GTADriving extends OpMode{
    //Declare Members
    private HardwareStarTrek robot = new HardwareStarTrek();

    //Initialization code
    @Override
    public void init() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Ben, Hopefully you can drive.");    //
    }

    //Unused loops
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    //ACTUAL DRIVING
    @Override
    public void loop() {

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        float forward = gamepad1.left_trigger; //Max:1
        float breaks = gamepad1.right_trigger; //Max:1
        telemetry.addData("Say", Float.toString(forward + breaks));
        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.backRight.setDirection(DcMotor.Direction.REVERSE);

        //Strafing controls
        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;
        final double v0 = r * Math.cos(robotAngle) + rightX;
        final double v1 = r * Math.sin(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.cos(robotAngle) - rightX;

        //Powers Motors
        robot.frontLeft.setPower(v0);
        robot.frontRight.setPower(v1);
        robot.backLeft.setPower(v2);
        robot.backRight.setPower(v3);

        //Hook control
        boolean Down = false;
        if(gamepad1.left_bumper) {
            telemetry.addData("Hook", robot.Hook.getPosition());
            robot.Hook.setPosition(0.06);
            Down = true;
        }
        if (gamepad1.right_bumper) {
            telemetry.addData("Hook", robot.Hook.getPosition());
            robot.Hook.setPosition(0.8);
            Down = false;
        }
        if (Down && robot.Hook.getPosition()!=0.06) {
            telemetry.addData("Hook", "Attempting to resist slippage");
            robot.Hook.setPosition(0.06);
        }

        //Actuator control
        if(gamepad1.dpad_down) {
            telemetry.addData("Actuator", robot.Actuator.getPosition());
            robot.Actuator.setPosition(1);
        }
        if(gamepad1.dpad_down == false) {
            telemetry.addData("Actuator", robot.Actuator.getPosition());
            robot.Actuator.setPosition(0);
        }
    }

    Runnable runnable = new Runnable() {
        @Override
        public void run() {
            int position = 1;
            int direction = 1;
            if (gamepad1.dpad_left) {
                position--;
            }
            if (gamepad1.dpad_right) {
                position++;
            }
            if (position == 0) {
                position = 3;
            }
            if (position == 4) {
                position = 1;
            }
            if (position == 1) {
                direction = 1;
                robot.FrontClaw.setPower(1);
            }
            if (position == 2 && direction == 1) {
                robot.FrontClaw.setPower(-1);
            }
            if (position == 2 && direction == 2) {
                robot.FrontClaw.setPower(1);
            }
            if (position == 3) {
                direction = 2;
                robot.FrontClaw.setPower(-1);
            }
        }
    };
    Thread thread = new Thread(runnable);

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
        public boolean isButtonDown(float gamePad) {
            if (gamePad == 0) return false;
            else return true;
        }
}

