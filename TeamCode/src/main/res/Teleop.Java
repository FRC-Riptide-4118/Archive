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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Robot: Teleop", group="Robot")
@Disabled
public class RobotTeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  frontleftWheel  = null;
    public DcMotor  rearrightWheel  = null;
    public DcMotor  frontrightWheel  = null;
    public DcMotor  rearleftWheel  = null;
    public DcMotor  leftintakeWheel  = null;
    public DcMotor  rearrightWheel  = null;

    
        // Define and Initialize Motors
        frontleftWheel  = hardwareMap.get(DcMotor.class, "front_left_wheel");
        rearleftWheel  = hardwareMap.get(DcMotor.class, "rear_left_wheel");
        frontrightWheel = hardwareMap.get(DcMotor.class, "front_right_wheel");
        rearrightWheel = hardwareMap.get(DcMotor.class, "rear_right_wheel");
    
        leftintakeWheel = hardwareMap.get(DcMotor.class, "left_intake_wheel");
        rightintakeWheel = hardwareMap.get(DcMotor.class, "right_intake_wheel");
    

        // Reversing the motors
        frontleftWheel.setDirection(DcMotor.Direction.REVERSE);
        rearleftWheel.setDirection(DcMotor.Direction.REVERSE);
        frontrightWheel.setDirection(DcMotor.Direction.FORWARD);
        rearrightWheel.setDirection(DcMotor.Direction.FORWARD);
    
        leftintakeWheel.setDirection(DcMotor.Direction.FORWARD);
        rightintakeWheel.setDirection(DcMotor.Direction.REVERSE);
    

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*-------Drivetrain-------*/
            
            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                frontleft /= max;
                frontright /= max;
                rearleft /= max;
                rearright /= max;
            }

            // Output the safe vales to the motor wheels.
            frontleftWheel.setPower(frontleft);
            frontrightWheel.setPower(frontright);
            rearleftWheel.setPower(rearleft);
            rearrightWheel.setPower(rearright);

            // Gamepad controls
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            frontleftWheel.setPower(v1);
            frontrightWheel.setPower(v2);
            rearleftWheel.setPower(v3);
            rearrightWheel.setPower(v4);
            
            /*-------INTAKE-------*/
            if (gamepad1.left_bumper)
            {
                leftintakewheel.setPower(.5)
                rightintakewheel.setPower(.5)
                }
            if else (gamepad1.right_bumper)
            {
                leftintakewheel.setPower(-.5)
                rightintakewheel.setPower(-.5)
                }
            else
            {
                leftintakewheel.setPower(0)
                rightintakewheel.setPower(0)
                }
                        
                
            
        }
    }
}
