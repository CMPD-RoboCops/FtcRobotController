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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="GTBTeleOp24", group="Linear OpMode")
//@Disabled
public class GTBTeleOp24 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontleft = null;
    private DcMotor backleft = null;
    private DcMotor rightfront = null;
    private DcMotor rightback = null;
    private Servo portarmservo = null;
    private Servo starboardarmservo = null;
    private Servo portbridgeservo = null; //Port bridge service
    private Servo starboardbridgeservo = null; //Starboard drone service
    private Servo droneservo = null; //Drone Servo
    private Servo portclawservo = null; //Port claw servo
    private Servo starboardclawservo = null; //Starboard claw servo
    private DcMotor intakemotor = null; //Intake motor
    private DcMotor armright = null; //Arm motor right
    private DcMotor armleft = null; //Arm motor left


    //Arm Position Variables
    double armmax = 8;
    double armposition=0;
    double armPower = 1;

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
        starboardarmservo = hardwareMap.get(Servo.class, "starboard arm servo");
        starboardbridgeservo = hardwareMap.get(Servo.class, "starboard bridge servo");
        portarmservo = hardwareMap.get(Servo.class, "port arm servo");
        portbridgeservo = hardwareMap.get(Servo.class, "port bridge servo");
        starboardclawservo = hardwareMap.get(Servo.class, "starboard claw servo");
        portclawservo = hardwareMap.get(Servo.class, "port claw servo");

        // Set Motor Directions
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.FORWARD);
        intakemotor.setDirection(DcMotor.Direction.REVERSE);
        armleft.setDirection(DcMotor.Direction.REVERSE);
        armright.setDirection(DcMotor.Direction.REVERSE);

        //Reversing of servos
        starboardbridgeservo.setDirection(Servo.Direction.REVERSE);
        portarmservo.setDirection(Servo.Direction.REVERSE);
        droneservo.setDirection(Servo.Direction.REVERSE);

        //Assign Controllers
        ServoControllerEx PortArmServoController = (ServoControllerEx) portarmservo.getController();
        ServoControllerEx StarboardArmServoController = (ServoControllerEx) starboardarmservo.getController();
        ServoControllerEx PortBridgeServoController = (ServoControllerEx) portarmservo.getController();
        ServoControllerEx StarboardBridgeServoController = (ServoControllerEx) starboardarmservo.getController();


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            double armPower = 1;

            //Both Bridge Servos in correct position at 0.2
            //Port Claw Servo in correct position at 0.4 to 0.5, but start it at 0.4
            //Starboard Claw Servo 0.86 to 0.91, start it at .91
            //Drone Servo...0.15 is the ready position, 0.3 is launch position
            //Arm Servos 0.9 is your scoring position

            portclawservo.setPosition(.46); //.46
            starboardclawservo.setPosition(.86); //.86
            droneservo.setPosition(.15);
            portarmservo.setPosition(.85);  //.85
            starboardarmservo.setPosition(.85); //.85

            if (gamepad1.a) //intake acquire
            {
                intakemotor.setPower(1);
                sleep(100);
                backleft.setPower(0);
                sleep(100);
            } else {
                intakemotor.setPower(0);
            }

            if(gamepad1.b) //attempt to get pixel
            {
                portclawservo.setPosition(.38);
                starboardclawservo.setPosition(.88);
                sleep(100);
                portarmservo.setPosition(0.65);
                starboardarmservo.setPosition(.65);
                portclawservo.setPosition(.41);
                starboardclawservo.setPosition(.89);
                sleep(750);
            }


            if(gamepad1.x) //arm up
            {
                PortArmServoController.setServoPwmEnable(portarmservo.getPortNumber());
                StarboardArmServoController.setServoPwmEnable(starboardarmservo.getPortNumber());
                PortBridgeServoController.setServoPwmEnable(portarmservo.getPortNumber());
                StarboardBridgeServoController.setServoPwmEnable(starboardbridgeservo.getPortNumber());

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

                starboardbridgeservo.setPosition(0.2);
                portbridgeservo.setPosition(0.2);
                portarmservo.setPosition(0.9);
                starboardarmservo.setPosition(0.9);
            }

            //Arm Retract
            if(gamepad1.y)
            {
                portarmservo.setPosition(.85);
                starboardarmservo.setPosition(.85);
                starboardbridgeservo.setPosition(1);
                portbridgeservo.setPosition(1);

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
                PortArmServoController.setServoPwmDisable(portarmservo.getPortNumber());
                StarboardArmServoController.setServoPwmDisable(starboardarmservo.getPortNumber());
                PortBridgeServoController.setServoPwmDisable(portarmservo.getPortNumber());
                StarboardBridgeServoController.setServoPwmDisable(starboardbridgeservo.getPortNumber());
            }

            if(gamepad1.left_bumper) //port agitate
            {
                portclawservo.setPosition(.38);
                sleep(100);
                portclawservo.setPosition(.4);
            }

            if(gamepad1.right_bumper) //starboard agitate
            {
                starboardclawservo.setPosition(.88);
                sleep(100);
                starboardclawservo.setPosition(.89);
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;


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