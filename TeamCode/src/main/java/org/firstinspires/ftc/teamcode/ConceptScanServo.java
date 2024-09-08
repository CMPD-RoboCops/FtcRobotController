package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
    static final double MAX_POS     =  0.3;     // Maximum rotational position
    static final double MIN_POS     =  0.5;     // Minimum rotational position

    //double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double position = 0.4;
    boolean rampUp = true;


    // Define class members

    private Servo portarmservo = null;  //Port Arm Servo
    private Servo starboardarmservo = null; //Starboard Arm Servo
    private Servo portbridgeservo = null; //Port bridge service
    private Servo starboardbridgeservo = null; //Starboard bridge service
    private Servo droneservo = null; //Drone Servo
    private Servo portclawservo = null; //Port claw servo
    private Servo starboardclawservo = null; //Starboard claw servo
    private Servo cameraservo = null; //Camera Servo
    private DcMotor armright = null; //Arm motor right
    private DcMotor armleft = null; //Arm motor left

    private DcMotor backleft = null;
    private DcMotor rightfront = null;
    private DcMotor rightback = null;

    private DcMotor frontleft = null;

    private DcMotor intakemotor = null; //Intake motor

    //Arm Position Variables
    double armmax = 8;
    double armposition=0;

    double counter = 0;


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
        cameraservo = hardwareMap.get(Servo.class, "camera servo");

        //Reversing of servos
        starboardbridgeservo.setDirection(Servo.Direction.REVERSE);
        portarmservo.setDirection(Servo.Direction.REVERSE);
        droneservo.setDirection(Servo.Direction.REVERSE);

        //Assign Controllers
        ServoControllerEx PortArmServoController = (ServoControllerEx) portarmservo.getController();
        ServoControllerEx StarboardArmServoController = (ServoControllerEx) starboardarmservo.getController();
        ServoControllerEx PortBridgeServoController = (ServoControllerEx) portarmservo.getController();
        ServoControllerEx StarboardBridgeServoController = (ServoControllerEx) starboardarmservo.getController();

        //Initialize Motors
        armright = hardwareMap.get(DcMotor.class, "arm right");
        armleft = hardwareMap.get(DcMotor.class, "arm left");
        intakemotor = hardwareMap.get(DcMotor.class, "intake motor");
        backleft  = hardwareMap.get(DcMotor.class, "back left");
        rightback = hardwareMap.get(DcMotor.class, "right back");
        frontleft  = hardwareMap.get(DcMotor.class, "front left");
        rightfront = hardwareMap.get(DcMotor.class, "right front");

        //Reversing Motors
        armleft.setDirection(DcMotor.Direction.REVERSE);
        armright.setDirection(DcMotor.Direction.REVERSE);
        intakemotor.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the start button
        telemetry.addData(">", "MJB Servo Test" );
        telemetry.update();
        waitForStart();

        // Scan servo till stop pressed.
            while(opModeIsActive()){

                // slew the servo, according to the rampUp (direction) variable.
                if (rampUp) {
                    // Keep stepping up until we hit the max value.
                    position += INCREMENT ;
                    telemetry.addData("Position: ",position);
                    telemetry.update();
                    if (position >= MAX_POS ) {
                        position = MAX_POS;
                        rampUp = !rampUp;   // Switch ramp direction
                    }
                    sleep(500);
                }
                else {
                    // Keep stepping down until we hit the min value.
                    position -= INCREMENT ;
                    telemetry.addData("Position: ",position);
                    telemetry.update();
                    if (position <= MIN_POS ) {
                        position = MIN_POS;
                        rampUp = !rampUp;  // Switch ramp direction
                    }
                    sleep(500);
                }

                telemetry.addData("Camera Servo Position: ",cameraservo.getPosition());
                telemetry.update();
                cameraservo.setPosition(position);

                double armPower = 1;

                //Both Bridge Servos in correct position at 0.2
                //Port Claw Servo in correct position at 0.4 to 0.5, but start it at 0.4
                //Starboard Claw Servo 0.86 to 0.91, start it at .91
                //Drone Servo...0.15 is the ready position, 0.3 is launch position
                //Arm Servos 0.9 is your scoring position

                portclawservo.setPosition(.43); //.46
                starboardclawservo.setPosition(.87); //.86
                droneservo.setPosition(.15);
                portarmservo.setPosition(.8);  //.85
                starboardarmservo.setPosition(.8); //.85

                if(gamepad1.a) //attempt to get pixel
                {
                    portclawservo.setPosition(.38);
                    starboardclawservo.setPosition(.89);
                    sleep(100);
                    portarmservo.setPosition(0.65);
                    starboardarmservo.setPosition(.65);
                    sleep(100);
                    portclawservo.setPosition(.4);
                    starboardclawservo.setPosition(.90);
                    sleep(750);
                }

                if (gamepad1.b) //intake acquire
                {
                    intakemotor.setPower(1);
                    backleft.setPower(0.5);
                    sleep(100);
                    backleft.setPower(0);
                    rightback.setPower(0.5);
                    sleep(100);
                    backleft.setPower(0);
                    rightback.setPower(0);
                } else {
                    intakemotor.setPower(0);
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
                    starboardclawservo.setPosition(.9);
                    sleep(100);
                    starboardclawservo.setPosition(.91);
                }


                sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
