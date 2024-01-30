package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Servo Test", group = "Test")
//@Disabled
public class ConceptScanServo extends LinearOpMode {


    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1;     // Maximum rotational position
    static final double MIN_POS     =  0;     // Minimum rotational position

    // Define class members

    private Servo portarmservo = null;
    private Servo starboardarmservo = null;
    private Servo portbridgeservo = null; //Port bridge service
    private Servo starboardbridgeservo = null; //Starboard drone service
    private Servo droneservo = null; //Drone Servo
    private Servo portclawservo = null; //Port claw servo
    private Servo starboardclawservo = null; //Starboard claw servo


    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    //double position = 1;
    boolean rampUp = true;


    @Override
    public void runOpMode() {

        //Initialize Servos
        droneservo = hardwareMap.get(Servo.class, "drone servo");
        starboardarmservo = hardwareMap.get(Servo.class, "starboard arm servo");
        starboardbridgeservo = hardwareMap.get(Servo.class, "starboard bridge servo");
        starboardclawservo = hardwareMap.get(Servo.class, "starboard claw servo");
        portarmservo = hardwareMap.get(Servo.class, "port arm servo");
        portbridgeservo = hardwareMap.get(Servo.class, "port bridge servo");
        portclawservo = hardwareMap.get(Servo.class, "port claw servo");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()){

            /*
            // slew the servo, according to the rampUp (direction) variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

             */

            //Drone Servo Test Code
            droneservo.setPosition(0.2);
            telemetry.addData("Drone Servo", "%5.2f", droneservo.getPosition());
            telemetry.update();
            sleep(1000);
            droneservo.setPosition(0.25);
            telemetry.addData("Drone Servo", "%5.2f", droneservo.getPosition());
            telemetry.update();
            sleep(1000);

            starboardarmservo.setPosition(0);

            /*
            //Bridge Positioning
            starboardbridgeservo.setPosition(.95);
            portbridgeservo.setPosition(0);
            telemetry.addData("Starboard Bridge", "%5.2f", starboardbridgeservo.getPosition());
            telemetry.addData("Port Bridge", "%5.2f", portbridgeservo.getPosition());
            telemetry.update();
            sleep(1000);
            starboardbridgeservo.setPosition(-1);
            portbridgeservo.setPosition(2);
            telemetry.addData("Starboard Bridge", "%5.2f", starboardbridgeservo.getPosition());
            telemetry.addData("POrt Bridge", "%5.2f", portbridgeservo.getPosition());
            telemetry.update();
            sleep(1000);


            /*
            //Arm Servo Test Code
            starboardarmservo.setPosition(1);
            telemetry.addData("Starboard Arm", "%5.2f", starboardarmservo.getPosition());
            telemetry.update();
            portarmservo.setPosition(1);
            telemetry.addData("Starboard Arm", "%5.2f", portarmservo.getPosition());
            telemetry.update();
            sleep(1000);
            */

            /*
            //Claw Servo Test Code
            starboardclawservo.setPosition(.6);  //Open
            telemetry.addData("Starboard Claw", "%5.2f", starboardclawservo.getPosition());
            telemetry.update();
            sleep(1000);
            starboardclawservo.setPosition(.57);  //Close
            telemetry.addData("Starboard Claw", "%5.2f", starboardclawservo.getPosition());
            telemetry.update();

            sleep(1000);

            portclawservo.setPosition(.2); //Open
            telemetry.addData("Port Claw", "%5.2f", portclawservo.getPosition());
            telemetry.update();
            sleep(1000);
            portclawservo.setPosition(.34);  //Close
            telemetry.addData("Port Claw", "%5.2f", portclawservo.getPosition());
            telemetry.update();
            sleep(1000);
            */
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
