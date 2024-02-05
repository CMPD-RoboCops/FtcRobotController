/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="GTBTeleOp24", group="Linear OpMode")
//@Disabled
public class GTBTeleOp24 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontleft = null;
    private DcMotor backleft = null;
    private DcMotor rightfront = null;
    private DcMotor rightback = null;
    private CRServo portarmservo = null;
    private CRServo starboardarmservo = null;
    private CRServo portbridgeservo = null; //Port bridge service
    private CRServo starboardbridgeservo = null; //Starboard drone service
    private Servo droneservo = null; //Drone Servo
    private Servo portclawservo = null; //Port claw servo
    private Servo starboardclawservo = null; //Starboard claw servo
    private DcMotor intakemotor = null; //Intake motor
    private DcMotor armright = null; //Arm motor right
    private DcMotor armleft = null; //Arm motor left


    //Arm Position Variables
    double armmax = 8;
    double armposition=0;

    //Servo General Variables
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle

    //Drone Servo Position Variables
    double DroneStartPosition = 0;
    double DroneCurrentPosition = 0;
    double DroneLaunchPosition=1;

    //Port Arm Servo Variables
    double PortArmCurrentPosition = 0;
    double PortArmStartPosition = 0;

    //Starboard Arm Servo
    double StarboardArmCurrentPosition = 0;
    double StarboardArmStartPosition = 0;

    //Port Arm Servo
    double PortBridgeCurrentPosition = 0;
    double PortBridgeStartPosition = 0;


    @Override
    public void runOpMode() {

        // Initialize Motors
        frontleft  = hardwareMap.get(DcMotor.class, "front left");
        backleft  = hardwareMap.get(DcMotor.class, "back left");
        rightfront = hardwareMap.get(DcMotor.class, "right front");
        rightback = hardwareMap.get(DcMotor.class, "right back");
        intakemotor = hardwareMap.get(DcMotor.class, "intake motor");
        armright = hardwareMap.get(DcMotor.class, "arm right");
        armleft = hardwareMap.get(DcMotor.class, "arm left");

        //Initialize Servos
        droneservo = hardwareMap.get(Servo.class, "drone servo");
        starboardarmservo = hardwareMap.get(CRServo.class, "starboard arm servo");
        starboardbridgeservo = hardwareMap.get(CRServo.class, "starboard bridge servo");
        portarmservo = hardwareMap.get(CRServo.class, "port arm servo");
        portbridgeservo = hardwareMap.get(CRServo.class, "port bridge servo");
        starboardclawservo = hardwareMap.get(Servo.class, "starboard claw servo");
        portclawservo = hardwareMap.get(Servo.class, "port claw servo");

        // Set Motor Directions
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.FORWARD);
        intakemotor.setDirection(DcMotor.Direction.FORWARD);
        armleft.setDirection(DcMotor.Direction.REVERSE);
        armright.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;



            //Set Motor Power Variables
            double intakePower = 0.1;
            double armPower = 1;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Button Mapping
            //boolean portarmservo = gamepad1.a;
            //boolean starboardarmservo = gamepad1.a;
            //boolean droneservo = gamepad1.dpad_up;
            boolean starboardclawservo = gamepad1.x;
            //boolean intakemotor = gamepad1.b;
            double armin = -gamepad1.left_trigger;
            double armout = gamepad1.right_trigger;

            /*if(gamepad2.a) {
                starboardbridgeservo.setPower(0.75);
                portbridgeservo.setPower(-0.75);
                sleep(500);
                starboardarmservo.setPower(-0.75);
                portarmservo.setPower(0.75);
                sleep(2000);
            } else if(gamepad2.b) {
                starboardarmservo.setPower(0.75);
                portarmservo.setPower(-0.75);
                sleep(500);
                starboardbridgeservo.setPower(-0.75);
                portbridgeservo.setPower(0.75);

            } else {
                starboardarmservo.setPower(0);
                portarmservo.setPower(0);
                starboardbridgeservo.setPower(0);
                portbridgeservo.setPower(0);
            }
            */
            //Arm Extend
            if(gamepad1.x)
            {
              while(armposition<armmax)
              {
                  armleft.setPower(-armPower);
                  armright.setPower(armPower);
                  sleep(100);
                  armleft.setPower(0);
                  armright.setPower(0);
                  armposition++;
              }
                armleft.setPower(-0.1);
                armright.setPower(0.1);
            }

            //Arm Retract
            if(gamepad1.y)
            {
                while(armposition>0)
                {
                    armleft.setPower(armPower);
                    armright.setPower(-armPower);
                    sleep(100);
                    armleft.setPower(0);
                    armright.setPower(0);
                    armposition--;
                }
                armleft.setPower(0);
                armright.setPower(0);
            }

            //Drone Launch
            if(gamepad1.dpad_up)
            {
                while(DroneCurrentPosition<DroneLaunchPosition)
                {
                    droneservo.setPosition(DroneCurrentPosition);
                    DroneCurrentPosition=DroneCurrentPosition+0.01;
                }
            }

            //Port Arm Servo
            if(gamepad2.a)
            {
                while (PortArmCurrentPosition < PortArmStartPosition)
                {
                    portarmservo.setPower(-0.75);
                }
            }
            else portarmservo.setPower(0);
                    //portarmservo.setPosition(PortArmCurrentPosition);
                    //PortArmCurrentPosition=PortArmCurrentPosition+0.01;



            //Starboard Arm Servo
            if(gamepad2.a) {
                while (StarboardArmCurrentPosition < StarboardArmStartPosition) {
                    starboardarmservo.setPower(-0.75);
                }
            }else starboardarmservo.setPower(0);
                    //starboardarmservo.setPosition(StarboardArmCurrentPosition);
                    //StarboardArmCurrentPosition=StarboardArmCurrentPosition+0.01;



            //Port Bridge Servo
            if(gamepad2.a)
            {
                portbridgeservo.setPower(-0.75);
            } else portbridgeservo.setPower(0);

            //Starboard Bridge
            if(gamepad2.a)
            {
                starboardbridgeservo.setPower(-0.75);
            } else starboardbridgeservo.setPower(0);

            //Intake Motor Temp Code
            if (gamepad1.b) {
                intakePower = 1.0;
            } else if (gamepad1.a) {
                intakePower = -1.0;
            } else {
                intakePower = 0.0;
            }

            intakemotor.setPower(intakePower);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */


            // Send calculated power to wheels
            frontleft.setPower(leftFrontPower);
            rightfront.setPower(rightFrontPower);
            backleft.setPower(leftBackPower);
            rightback.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("frontleft/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            //telemetry.addData("Arm Position/Arm Max","%4.2f, %4.2f", armposition, armmax);
            telemetry.update();
        }
    }}