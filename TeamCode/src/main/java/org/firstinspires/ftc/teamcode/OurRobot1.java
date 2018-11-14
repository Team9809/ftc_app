/*
Copyright (c) 2016 Robert Atkinson
        All rights reserved.
        Redistribution and use in source and binary forms, with or without modification,
        are permitted (subject to the limitations in the disclaimer below) provided that
        the following conditions are met:
        Redistributions of source code must retain the above copyright notice, this list
        of conditions and the following disclaimer.
        Redistributions in binary form must reproduce the above copyright notice, this
        list of conditions and the following disclaimer in the documentation and/or
        other materials provided with the distribution.
        Neither the name of Robert Atkinson nor the names of his contributors may be used to
        endorse or promote products derived from this software without specific prior
        written permission.
        NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
        LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
        "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
        THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
        ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
        FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
        DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
        SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
        CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
        TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
        THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
        */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is our COMPETITION teleOp program
 *
 * It uses two controllers: 1 for driving, 2 for controlling arm and servo
 * Adding encoders to both arm motors to allow for position hold and min && max thresholds
 *
 */

@TeleOp(name="compTele: OurRobot1 ", group="compTele")  // @Autonomous(...) is the other common choice
//@Disabled
public class OurRobot1 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //motors
    DcMotor motorRight = null;
    DcMotor motorLeft = null;
    DcMotor motorArm2 = null;
    DcMotor motorArm1 = null;

    //servos
    //Servo servoHandL = null;
    //Servo servoHandR = null;

    //colector
    Servo servo1 = null;

    //Create and set default hand positions variables. To be determined based on your build
    double counterclockwise = 0.2;
    double clockwise = 0.8;
    double stop = 0.5;

    // variables for arm limits and hold position
    // note: these can be placed in your hardwareSetup Class
    // ( not currently using min/max values for arm operation
    double  arm1MinPos        = 0.0;      // encoder position for arm at bottom
    double  arm2MinPos        = 0.0;      // encoder position for arm at bottom
    double  arm1MaxPos        = 5380.0;   // encoder position for arm at top
    double  arm2MaxPos        = 5380.0;   // encoder position for arm at top
    int     arm1HoldPosition;             // reading of arm position when buttons released to hold
    int     arm2HoldPosition;             // reading of arm position when buttons released to hold
    double  slopeVal         = 5000.0;   // increase or decrease to perfect


    @Override
    public void runOpMode() throws InterruptedException {
        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        motorLeft  = hardwareMap.dcMotor.get("motorL");
        motorRight = hardwareMap.dcMotor.get("motorR");
        motorArm2 = hardwareMap.dcMotor.get("motorArm2");
        motorArm1 = hardwareMap.dcMotor.get("motorArm1");
        //servoHandL = hardwareMap.servo.get("servoHandL"); //assuming a pushBot configuration of two servo grippers
        servo1 = hardwareMap.servo.get("servo1");

        // eg: Set the drive motor directions:
        //"Reverse" the motor that runs backwards when connected directly to the battery
        motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorArm1.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration
        motorArm2.setDirection (DcMotor.Direction.FORWARD);

        // set arm motors to use encoders
        motorArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

          // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /************************
         * TeleOp Code Below://
         *************************/
        arm1HoldPosition = motorArm1.getCurrentPosition();
        arm2HoldPosition = motorArm2.getCurrentPosition();

        while (opModeIsActive()) {  // run until the end of the match (driver presses STOP)

    //init current position of arm motor

            //Display runtime and arm positions
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("armPostion: ", + motorArm1.getCurrentPosition());
            telemetry.addData("armPostion: ", + motorArm2.getCurrentPosition());
            telemetry.update();

    // tank drive set to gamepad1 joysticks
            //(note: The joystick goes negative when pushed forwards)
            motorLeft.setPower(gamepad1.left_stick_y);
            motorRight.setPower(gamepad1.right_stick_y);

    // Arm Control - Uses dual buttons (bumper && trigger) to control motor direction

            //Control motorArm1 - the shoulder
            if(gamepad2.right_bumper)
            {
                motorArm1.setPower(-gamepad2.right_trigger); // if both Bumper + Trigger, then negative power, runs arm down
                arm1HoldPosition = motorArm1.getCurrentPosition(); // continuously update hold position when moving arm
            }
            else if (!gamepad2.right_bumper)
            {
                motorArm1.setPower(gamepad2.right_trigger);  // else trigger positive value, runs arm up
                arm1HoldPosition = motorArm1.getCurrentPosition(); // continuously update hold position when moving arm
            }
            else // else runs motor to hold current position
            {
                motorArm1.setPower((double) (arm1HoldPosition - motorArm1.getCurrentPosition()) / slopeVal);
            }

            //Control motorArm2 - the elbow
            if(gamepad2.left_bumper)
            {
                motorArm2.setPower(-gamepad2.left_trigger); // if both Bumper + Trigger, then negative power, runs arm down
                arm2HoldPosition = motorArm2.getCurrentPosition(); // continuously update hold position when moving arm
            }
            else if(!gamepad2.left_bumper)
            {
                motorArm2.setPower(gamepad2.left_trigger);  // else trigger positive value, runs arm up
                arm2HoldPosition = motorArm2.getCurrentPosition(); // continuously update hold position when moving arm
            }
            else // else runs motor to hold current position
            {
                motorArm2.setPower((double) (arm2HoldPosition - motorArm2.getCurrentPosition()) / slopeVal);
            }


    //servo commands
            if(gamepad2.a) //button 'a' runs CR servo CW
            {
                servo1.setPosition(clockwise);

            }
            else if (gamepad2.b) //button 'b' runs CR servo CCW
            {
                servo1.setPosition(counterclockwise);

            }
            else if (gamepad2.x) // button 'x' stops CR servo
            {
                servo1.setPosition(stop);
            }


            //idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}