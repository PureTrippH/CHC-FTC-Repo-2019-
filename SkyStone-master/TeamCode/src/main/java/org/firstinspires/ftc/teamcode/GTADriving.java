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
public class GTADriving extends OpMode {
    //Declare OpMode Members
    private HardwareStarTrek robot = new HardwareStarTrek();

    //Run on init
    @Override
    public void init() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Ben, Hopefully you can drive.");    //
    }

    //Future Stuff
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

        //Strafing
        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;
        final double v0 = r * Math.cos(robotAngle) + rightX;
        final double v1 = r * Math.sin(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.cos(robotAngle) - rightX;

        //Sets Power to Motors
        robot.frontLeft.setPower(v0);
        robot.frontRight.setPower(v1);
        robot.backLeft.setPower(v2);
        robot.backRight.setPower(v3);

        //Front Claw Control
        if (gamepad1.left_bumper) {
            robot.FrontClaw.setPosition(robot.FrontClaw.getPosition() + 0.001);
            telemetry.addData("Front Claw", robot.FrontClaw.getPosition());
        }

        //Hook Control
        if (gamepad1.right_bumper) {
            robot.Hook.setPosition(robot.Hook.getPosition() + 0.02);
            telemetry.addData("Hook", robot.FrontClaw.getPosition());
        }
    }
}