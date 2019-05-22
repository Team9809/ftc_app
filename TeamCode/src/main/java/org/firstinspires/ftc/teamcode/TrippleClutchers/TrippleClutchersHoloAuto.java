/**
 * This file contains an minimal example of a Linear Autonomous "OpMode".
 *
 * This particular OpMode just executes a basic Autonomous driving for time, not using encoders
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Don't forget to comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
package org.firstinspires.ftc.teamcode.TrippleClutchers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="TrippleHolo", group="comp")  // @TeleOp(...) is the other common choice
//@Disabled
public class TrippleClutchersHoloAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    HardwareTrippleClutchers robot = new HardwareTrippleClutchers();

    int     shoulderHoldPosition;
    int     elbowHoldPosition;
    double  slopeVal            = 2000.0;
    double  clockwise           = 0.2;
    double  counterclockwise    = 0.8;
    double  stop                = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        robot.init(hardwareMap);  //Initialize hardware from the Hardware Setup Class

        shoulderHoldPosition = robot.motorShoulder.getCurrentPosition();
        elbowHoldPosition = robot.motorElbow.getCurrentPosition();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /************************
         * Autonomous Code Below://
         *************************/
        DriveForwardTime(0.6, 2500);
        StopDrivingTime(500);


       

    }//runOpMode

    /** Below: Basic Drive Methods used in Autonomous code...**/
    //set Drive Power variable
    double DRIVE_POWER = 1.0;

    public void DriveForward(double power)
    {
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackLeft.setPower(power);
        robot.motorBackRight.setPower(power);
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
        robot.motorFrontLeft.setPower(-power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackLeft.setPower(-power);
        robot.motorBackRight.setPower(power);
        Thread.sleep(time);
    }

    public void TurnRight(double power, long time) throws InterruptedException
    {
        TurnLeft(-power, time);
    }

    /**
     * This file contains an minimal example of a Linear Tele "OpMode".
     *
     * This particular OpMode just executes a basic Tank Drive, Arm and 2 Servos for a PushBot
     * It includes all the skeletal structure that all linear OpModes contain.
     *
     * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
     * Comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     */

    @TeleOp(name="Example: TeleOp", group="Examples")  // @Autonomous(...) is the other common choice
    @Disabled
    public static class Example_TeleOp extends LinearOpMode {

        /* Declare OpMode members. */
        private ElapsedTime runtime = new ElapsedTime();
        //motors
        DcMotor motorLeft = null;
        DcMotor motorRight = null;
        DcMotor motorArm = null;

        //servos
        Servo servoHandL = null;
        Servo servoHandR = null;

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
             motorArm = hardwareMap.dcMotor.get("motorArm");
             servoHandL = hardwareMap.servo.get("servoHandL"); //assuming a pushBot configuration of two servo grippers
             servoHandR = hardwareMap.servo.get("servoHandR");

            // eg: Set the drive motor directions:
            // "Reverse" the motor that runs backwards when connected directly to the battery
             motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
             motorRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
             motorArm.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration

            //Set servo hand grippers to open position.
             servoHandL.setPosition(OPEN);
             servoHandR.setPosition(OPEN);

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            /************************
             * TeleOp Code Below://
             *************************/

            while (opModeIsActive()) {  // run until the end of the match (driver presses STOP)
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();

                // tank drive set to gamepad1 joysticks
                //(note: The joystick goes negative when pushed forwards)
                motorLeft.setPower(gamepad1.left_stick_y);
                motorRight.setPower(gamepad1.right_stick_y);

                // Arm Control - Uses dual buttons to control motor direction
                if(gamepad1.right_bumper)
                {
                    motorArm.setPower(-gamepad1.right_trigger); // if both Bumper + Trigger, then negative power, runs arm down
                }
                else
                {
                    motorArm.setPower(gamepad1.right_trigger);  // else trigger positive value, runs arm up
                }

                //servo commands
                if(gamepad1.a) //button 'a' will open
                {
                    servoHandR.setPosition(OPEN);
                    servoHandL.setPosition(OPEN);
                }
                else if (gamepad1.b) //button 'b' will close
                {
                    servoHandR.setPosition(CLOSED);
                    servoHandL.setPosition(CLOSED);
                }


                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
        }
    }
}//TestAutoDriveByTime
