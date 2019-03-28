package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ExampleCode.HardwareSetupHolonomicExample;


/**
 *
 * This is a Linear version program (i.e. uses runOpMode() and waitForStart() methods,  instead of init, loop and stop)
 * for TeleOp control with a single controller
 */

/*
   Holonomic concepts from:
   http://www.vexforum.com/index.php/12370-holonomic-drives-2-0-a-video-tutorial-by-cody/0
   Robot wheel mapping:
          X FRONT X
        X           X
      X  FL       FR  X
              X
             XXX
              X
      X  BL       BR  X
        X           X
          X       X
*/
@TeleOp(name = "compTele: TrippleClutchersHolonomics", group = "compTele")
//@Disabled
public class TrippleClutchersHolonomic extends LinearOpMode
{

    // create timer
    private ElapsedTime runtime = new ElapsedTime();

    //  DON'T FORGET TO RENAME HARDWARE CONFIG FILE NAME HERE!!!!!!
    HardwareTrippleClutchers robot = new HardwareTrippleClutchers();

    int     shoulderHoldPosition;
    int     elbowHoldPosition;
    double  slopeVal            = 2000.0;
    double  clockwise           = 0.2;
    double  counterclockwise    = 0.8;
    double  stop                = 0.53;

    @Override
    public void runOpMode() throws InterruptedException
    {
        /*
         * Use the hardwareMap to get the dc motors and servos by name. Note
         * that the names of the devices must match the names used when you
         * configured your robot and configuration file.
         */
        robot.init(hardwareMap);  //Initialize hardware from the Hardware Setup Class

        shoulderHoldPosition = robot.motorShoulder.getCurrentPosition();
        elbowHoldPosition = robot.motorElbow.getCurrentPosition();

        waitForStart();
        runtime.reset(); // starts timer once start button is pressed

        while(opModeIsActive())
        {
            // left stick: X controls Strafe & Y controls Spin Direction
            // right stick: Y controls drive Forward/Backward
            float gamepad1LeftY = -gamepad1.left_stick_y;   // drives spin left/right
            float gamepad1LeftX = gamepad1.left_stick_x;    // strafe direction (side to side)
            float gamepad1RightY = gamepad1.right_stick_y;  //drives forwards and backwards

            // holonomic formulas
            float FrontLeft  = -gamepad1LeftY - gamepad1LeftX - gamepad1RightY;
            float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightY;
            float BackRight  = gamepad1LeftY + gamepad1LeftX - gamepad1RightY;
            float BackLeft   = -gamepad1LeftY + gamepad1LeftX - gamepad1RightY;

            // clip the right/left values so that the values never exceed +/- 1
            FrontRight = Range.clip(FrontRight, -1, 1);
            FrontLeft = Range.clip(FrontLeft, -1, 1);
            BackLeft = Range.clip(BackLeft, -1, 1);
            BackRight = Range.clip(BackRight, -1, 1);

            // write the clipped values from the formula to the motors
            robot.motorFrontRight.setPower(FrontRight);
            robot.motorFrontLeft.setPower(-FrontLeft);
            robot.motorBackLeft.setPower(-BackLeft);
            robot.motorBackRight.setPower(BackRight);

            /*
             * Display Telemetry for debugging
             */

            telemetry.addData("Text", "*** Robot Data***");
            telemetry.addData("Joy XL YL XR", String.format("%.2f", gamepad1LeftX) + " " + String.format("%.2f", gamepad1LeftY) + " " + String.format("%.2f", gamepad1RightY));
            telemetry.addData("f left pwr", "front left  pwr: " + String.format("%.2f", FrontLeft));
            telemetry.addData("f right pwr", "front right pwr: " + String.format("%.2f", FrontRight));
            telemetry.addData("b right pwr", "back right pwr: " + String.format("%.2f", BackRight));
            telemetry.addData("b left pwr", "back left pwr: " + String.format("%.2f", BackLeft));
            telemetry.addData( "status", "runtime:" + runtime.toString());
            telemetry.addData("shoulderPostion: ", + robot.motorShoulder.getCurrentPosition());
            telemetry.addData("shoulderHoldPos: ", + shoulderHoldPosition);
            telemetry.addData("ElbowPostion: ", + robot.motorElbow.getCurrentPosition());
            telemetry.addData("ElbowHoldPostion: ", + elbowHoldPosition);

        //Control Shoulder
            if(gamepad2.right_bumper && gamepad2.right_trigger > 0.2)//using 0.2 instead of 0.0 as a threshold in case the trigger does not fully release
            {

                robot.motorShoulder.setPower(-gamepad2.right_trigger); // if both Bumper + Trigger, then negative power, runs arm down
                shoulderHoldPosition = robot.motorShoulder.getCurrentPosition(); // continuously update hold position when moving arm
            }
            else if (!gamepad2.right_bumper && gamepad2.right_trigger > 0.2)
            {
                robot.motorShoulder.setPower(gamepad2.right_trigger);  // else trigger positive value, runs arm up
               shoulderHoldPosition = robot.motorShoulder.getCurrentPosition(); // continuously update hold position when moving arm
            }
            else // else runs motor to hold current position
            {
                robot.motorShoulder.setPower((double) (shoulderHoldPosition - robot.motorShoulder.getCurrentPosition()) / slopeVal);
            }

            //Control the elbow
            if(gamepad2.left_bumper && gamepad2.left_trigger > 0.2)
            {
                robot.motorElbow.setPower(-gamepad2.left_trigger); // if both Bumper + Trigger, then negative power, runs arm down
                elbowHoldPosition = robot.motorElbow.getCurrentPosition(); // continuously update hold position when moving arm
            }
            else if(!gamepad2.left_bumper && gamepad2.left_trigger > 0.2)
            {
                robot.motorElbow.setPower(gamepad2.left_trigger);  // else trigger positive value, runs arm up
                elbowHoldPosition = robot.motorElbow.getCurrentPosition(); // continuously update hold position when moving arm
            }
            else // else runs motor to hold current position
            {
                robot.motorElbow.setPower((double) (elbowHoldPosition - robot.motorElbow.getCurrentPosition()) / slopeVal);
            }


            //servo commands
            if(gamepad2.a) //button 'a' runs CR servo CW
            {
                robot.handServo.setPosition(clockwise);

            }
            else if (gamepad2.b) //button 'b' runs CR servo CCW
            {
                robot.handServo.setPosition(counterclockwise);

            }
            else if (gamepad2.x) // button 'x' stops CR servo
            {
                robot.handServo.setPosition(stop);
            }
        }
    }
}