package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;


@TeleOp(name = "Servo Test", group = "Test")
//@Disabled
public class ConceptScanServo extends LinearOpMode {

    //Both Bridge Servos in correct position at 0.2
    //Port Arm Servo in correct position at 0.4 to 0.5, but start it at 0.4
    //Starboard Arm Servo 0.86 to 0.91, start it at .91
    //Drone Servo...0.15 is the ready position, 0.3 is launch position
    //Arm Servos 0.9 is your scoring position

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.9;     // Maximum rotational position
    static final double MIN_POS     =  0.8;     // Minimum rotational position

    //double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double position = 0.9;
    boolean rampUp = true;


    // Define class members

    private Servo portarmservo = null;  //Port Arm Servo
    private Servo starboardarmservo = null; //Starboard Arm Servo
    private Servo portbridgeservo = null; //Port bridge service
    private Servo starboardbridgeservo = null; //Starboard bridge service
    private Servo droneservo = null; //Drone Servo
    private Servo portclawservo = null; //Port claw servo
    private Servo starboardclawservo = null; //Starboard claw servo
    private DcMotor armright = null; //Arm motor right
    private DcMotor armleft = null; //Arm motor left

    //Arm Position Variables
    double armmax = 8;
    double armposition=0;



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

        //Reversing of servos
        starboardbridgeservo.setDirection(Servo.Direction.REVERSE);
        portarmservo.setDirection(Servo.Direction.REVERSE);
        droneservo.setDirection(Servo.Direction.REVERSE);

        //Assign Controllers
        ServoControllerEx PortArmServoController = (ServoControllerEx) portarmservo.getController();
        ServoControllerEx StarboardArmServoController = (ServoControllerEx) starboardarmservo.getController();
        ServoControllerEx PortBridgeServoController = (ServoControllerEx) portarmservo.getController();
        ServoControllerEx StarboardBridgeServoController = (ServoControllerEx) starboardarmservo.getController();

        //Initialize Arm Motors
        armright = hardwareMap.get(DcMotor.class, "arm right");
        armleft = hardwareMap.get(DcMotor.class, "arm left");

        //Reversing Motors
        armleft.setDirection(DcMotor.Direction.REVERSE);
        armright.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the start button
        telemetry.addData(">", "MJB Servo Test" );
        telemetry.update();
        waitForStart();

        //starboardbridgeservo.setPosition(0.2);
        //portbridgeservo.setPosition(0.2);


        // Scan servo till stop pressed.
            while(opModeIsActive()){

                // slew the servo, according to the rampUp (direction) variable.
                if (rampUp) {
                    // Keep stepping up until we hit the max value.
                    position += INCREMENT ;
                    //telemetry.addData("Position: ",position);
                    //telemetry.update();
                    if (position >= MAX_POS ) {
                        position = MAX_POS;
                        rampUp = !rampUp;   // Switch ramp direction
                    }
                }
                else {
                    // Keep stepping down until we hit the min value.
                    position -= INCREMENT ;
                    //telemetry.addData("Position: ",position);
                    //telemetry.update();
                    if (position <= MIN_POS ) {
                        position = MIN_POS;
                        rampUp = !rampUp;  // Switch ramp direction
                    }
                }

                // Display the current value
                //telemetry.addData("Servo Position", "%5.2f", position);
                //telemetry.addData(">", "Press Stop to end test." );
                //telemetry.update();

                // Set the servo to the new position and pause;

                //starboardarmservo.setPosition(position);
                //portarmservo.setPosition(position);

                //droneservo.setPosition(position);

                //telemetry.addData("Drone Position", "%5.2f", droneservo.getPosition());

                //telemetry.addData("Starboard Position", "%5.2f", starboardarmservo.getPosition());
                //telemetry.addData("Port Position     ","%5.2f",portarmservo.getPosition());
                //telemetry.update();

                double armPower = 1;

                //Both Bridge Servos in correct position at 0.2
                //Port Claw Servo in correct position at 0.4 to 0.5, but start it at 0.4
                //Starboard Claw Servo 0.86 to 0.91, start it at .91
                //Drone Servo...0.15 is the ready position, 0.3 is launch position
                //Arm Servos 0.9 is your scoring position

                portclawservo.setPosition(.46);
                starboardclawservo.setPosition(.86);
                droneservo.setPosition(.15);
                portarmservo.setPosition(.85);
                starboardarmservo.setPosition(.85);

                if(gamepad1.x)
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

                if(gamepad1.a)
                {
                    portclawservo.setPosition(0.35);
                    starboardclawservo.setPosition(.91);
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

                sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
