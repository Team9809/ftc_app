/**
 * This file contains an minimal example of a Linear Autonomous "OpMode".
 *
 * This particular OpMode just executes a basic Autonomous driving for time, not using encoders
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Don't forget to comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * This is our COMPETITION Autonomous program
 *
 * It uses time to set drive and turn amounts
 */

@Autonomous(name="Auto1", group="Examples")  // @TeleOp(...) is the other common choice
//@Disabled
public class OurAutoDriveByTime1 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //motors
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor motorArm2 = null;
    DcMotor motorArm1 = null;
    Servo bucketServo = null;



    //Create and set default hand positions variables. To be determined based on your build
    double CLOSED = 0.2;
    double OPEN = 0.8;

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
        //servo1 = hardwareMap.servo.get("servo1"); //assuming a pushBot configuration of two servo grippers
        bucketServo = hardwareMap.servo.get("servo1");
        
        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
         motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
         motorRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
         motorArm1.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration
        motorArm2.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration

        //Set bucketServo to stop
        bucketServo.setPosition(0.5);

        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /************************
         * Autonomous Code Below://  Use the basic drive methods at the bottom of this program to control robot movement
         *************************/



        DriveForwardTime(1, 3250);






        /*****************************
         * End of Autonomous Code
         ****************************/

    }//runOpMode

/**
 *  Below: Basic Drive Methods used in Autonomous code...
 */

    //set Drive Power variable
    double DRIVE_POWER = 1.0;

    public void DriveForward(double power)
    {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public void DriveForwardTime(double power, long time) throws InterruptedException
    {
        DriveForward(power);
        Thread.sleep(time);
    }

    public void StopDriving()
    {
        DriveForward(0);
    }

    public void StopDrivingTime(long time) throws InterruptedException
    {
        DriveForwardTime(0, time);
    }

    public void TurnLeft(double power, long time) throws InterruptedException
    {
        motorLeft.setPower(-power);
        motorRight.setPower(power);
        Thread.sleep(time);
    }

    public void TurnRight(double power, long time) throws InterruptedException
    {
        TurnLeft(-power, time);
    }

    //Sweeper servo
    public void DropBlock() throws InterruptedException
    {
        bucketServo.setPosition(0.8); // may need to reverse servo setting to drive correct direction

    }

    public void PickupBlock() throws InterruptedException
    {
        bucketServo.setPosition(0.3);

    }


}//TestAutoDriveByTime
