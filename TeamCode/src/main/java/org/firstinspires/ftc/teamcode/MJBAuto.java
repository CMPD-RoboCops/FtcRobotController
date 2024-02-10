package org.firstinspires.ftc.teamcode;

//Road Runner Imports
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

//TensorFlow Imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//April Tag Imports
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//Sensor Imports
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//General Imports
import java.util.List;

@Config
@Autonomous(name= "Red Back Auto",group = "drive")
public class MJBAuto extends LinearOpMode
{
    //TensorFlow Setup
    private static final boolean USE_WEBCAM = true;

    //Change Me for which alliance we are programming for
    private static final String TFOD_MODEL_ASSET = "RedCarModelv2.tflite";
    //private static final String TFOD_MODEL_ASSET = "BlueCarModel.tflite";
    private static final String[] LABELS = {
            "C","Car"
    };

    //Init TensorFlow, April Tag detection, and Vision Portal
    private TfodProcessor tfod;

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    //Init Sensors
    private DistanceSensor sensorDistancePort;
    private DistanceSensor sensorDistanceStarboard;
    private ColorSensor colorPort;
    private ColorSensor colorStarboard;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.45;     // Maximum rotational position
    static final double MIN_POS     =  0.27;     // Minimum rotational position
    Servo servo;
    double position = .38;
    boolean rampUp = true;

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
    public void runOpMode() throws InterruptedException
    {
        //Initiate TensorFlow
        initTfod();

        //Update Telemetry
        telemetry.addData(">", "Touch Play to start Autonomous");
        telemetry.update();

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


        //RoadRunner Initiation
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Setup Sensors
        sensorDistancePort = hardwareMap.get(DistanceSensor.class, "distancesensorport");
        sensorDistanceStarboard = hardwareMap.get(DistanceSensor.class, "distancesensorstarboard");

        servo = hardwareMap.get(Servo.class, "camera servo");

        colorPort = hardwareMap.get(ColorSensor.class, "colorsensorport");
        colorStarboard = hardwareMap.get(ColorSensor.class, "colorsensorstarboard");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistancePort;
        Rev2mDistanceSensor sensorTimeOfFlightStarboard = (Rev2mDistanceSensor) sensorDistanceStarboard;

        //Define Your Trajectories Here
        //TODO TUNE ALL
        //NOTE Using Trajectory Sequence now.
        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence ToCenterSpike = drive.trajectorySequenceBuilder(startPose)
                .forward(36.5) // Tune Me
                .build();

        TrajectorySequence CenterSpikeToBoard = drive.trajectorySequenceBuilder(startPose)
                .back(6)
                .turn(-110)
                .forward(35)
                .build();

        TrajectorySequence ToLeftSpike = drive.trajectorySequenceBuilder(startPose)
                .forward(32)
                .turn(110)
                .forward(12.5)
                .build();

        TrajectorySequence LeftSpikeToBoard = drive.trajectorySequenceBuilder(startPose)
                .back(20)
                .turn(-220)
                .forward(35)
                .build();

        TrajectorySequence ToRightSpike = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeTo(new Vector2d(27, -13)) // Tune Me
                .build();

        TrajectorySequence RightSpikeToBoard = drive.trajectorySequenceBuilder(new Pose2d())
                .back(10)
                .turn(-110)
                .forward(15)
                .strafeLeft(15)
                .forward(10)
                .build();

        TrajectorySequence AprilTagSlideRight = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(7.5)
                .build();

        TrajectorySequence AprilTagSlideLeft = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(7.5)
                .build();

        TrajectorySequence LeftPark = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(42)
                .build();

        TrajectorySequence CenterPark = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(36)
                .build();

        TrajectorySequence RightPark = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(30)
                .build();

        //Control Structure Variables
        int TeamPropPosition = 0;
        boolean SpikeLineFound = false;

        //Debug Code
        //telemetry.addData(">", "Checkpoint #1");
        //telemetry.update();

        //Very last thing before OpModeIsActive
        waitForStart();

        portclawservo.setPosition(.46);
        starboardclawservo.setPosition(.86);

        if (opModeIsActive())
        {
            telemetryTfod();

            servo.setPosition(MAX_POS);

            while(tfod.getRecognitions().size()==0)
            {
                telemetry.addData("Image", tfod.getRecognitions().size());
                if(rampUp) {
                    servo.setPosition(MAX_POS);
                    rampUp = !rampUp;
                } else {
                    servo.setPosition(MIN_POS);
                    rampUp = !rampUp;
                    sleep(100);
                }
                sleep(1200);
            }

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            for (Recognition recognition : currentRecognitions)
            {
                double CarX = (recognition.getLeft() + recognition.getRight()) / 2;
                telemetry.addData("X Position", CarX);
                if (CarX < 350 && !rampUp)
                {
                    TeamPropPosition = 4;
                    telemetry.addData("Line", TeamPropPosition);
                }
                else if ((CarX > 350 && !rampUp) || (CarX < 180 && rampUp))
                {
                    TeamPropPosition = 5;
                    telemetry.addData("Line", TeamPropPosition);
                }
                else if (CarX > 180 && rampUp)
                {
                    TeamPropPosition = 6;
                    telemetry.addData("Line", TeamPropPosition);
                }
                else
                {
                    telemetry.addData("Line", "Unknown");
                    telemetry.update();
                }
            }

            // Push telemetry to the Driver Station and turn off streaming
            telemetry.update();
            //visionPortal.stopStreaming();

            // CENTER
            if(TeamPropPosition == 5)
            {
                //Move up to SpikeLine, should overshoot a slight amount
                drive.followTrajectorySequence(ToCenterSpike);
                portclawservo.setPosition(0.35);
                drive.followTrajectorySequence(CenterSpikeToBoard);
            }
            //LEFT
            //TODO Move Properly with RoadRunner
            if(TeamPropPosition == 4)
            {
                //Move up to SpikeLine, should overshoot a slight amount
                drive.followTrajectorySequence(ToLeftSpike);
                portclawservo.setPosition(0.35);
                drive.followTrajectorySequence(LeftSpikeToBoard);
                /*
                // May not need to crawl back, Roadrunner might be accurate enough.
                drive.followTrajectory(SideSpikeForward);
                drive.followTrajectory(LeftSpikeTurn);
                drive.followTrajectory(SideSpikeOvershoot);
                while (SpikeLineFound == false)
                {
                    if (colorStarboard.red() > 400 || colorPort.red() > 400) {
                        SpikeLineFound = true;
                        // Drop Spike Code Here
                        drive.followTrajectory(LeftSpikeRecover);
                        drive.followTrajectory(TurnAround);
                    } else {
                        drive.followTrajectory(BackwardCreep);
                    }
                }*/
            }
            //RIGHT
            //TODO Move Properly with RoadRunner
            if(TeamPropPosition == 6)
            {
                //Move up to SpikeLine, should overshoot a slight amount
                drive.followTrajectorySequence(ToRightSpike);
                portclawservo.setPosition(0.35);
                drive.followTrajectorySequence(RightSpikeToBoard);
                /*
                //May not need to crawl back, Roadrunner might be accurate enough.
                drive.followTrajectory(SideSpikeForward);
                drive.followTrajectory(RightSpikeTurn);
                drive.followTrajectory(SideSpikeOvershoot);
                while (SpikeLineFound == false)
                drive.turn(Math.toRadians(110));
                drive.followTrajectory(SideSpikeOvershoot);
                {
                    if (colorStarboard.red() > 400 || colorPort.red() > 400) {
                        SpikeLineFound = true;
                        // Drop Pixel Code Here
                        drive.followTrajectory(LeftSpikeRecover);
                        drive.followTrajectory(RightSpikeSlideRight);
                        drive.followTrajectory(RightSpikeForward);
                        drive.followTrajectory(RightSpikeSlideLeft);
                    } else {
                        drive.followTrajectory(BackwardCreep);
                    }
                }
                 */
            }

            //Move to backdrop
            //TODO May need to be re-added if Roadrunner is not consistent enough
            /*
            double portDistance = sensorDistancePort.getDistance(DistanceUnit.INCH);
            double starboardDistance = sensorDistanceStarboard.getDistance(DistanceUnit.INCH);
            double averageDistance = (portDistance + starboardDistance) / 2;

            telemetry.addData("Port range", String.format("%.01f in", portDistance));
            telemetry.addData("Starboard range", String.format("%.01f in",starboardDistance));
            telemetry.update();

            while(averageDistance>30)
            {
                drive.followTrajectory(FastCrawl);
                portDistance = sensorDistancePort.getDistance(DistanceUnit.INCH);
                starboardDistance = sensorDistanceStarboard.getDistance(DistanceUnit.INCH);
                averageDistance = (portDistance + starboardDistance) / 2;

                //Update Telemetry
                telemetry.addData("Port range", String.format("%.01f in", portDistance));
                telemetry.addData("Starboard range", String.format("%.01f in",starboardDistance));
                telemetry.update();
            }

            while(averageDistance>15)
            {
                drive.followTrajectory(FastCrawl);
                portDistance = sensorDistancePort.getDistance(DistanceUnit.INCH);
                starboardDistance = sensorDistanceStarboard.getDistance(DistanceUnit.INCH);
                averageDistance = (portDistance + starboardDistance) / 2;

                //Update Telemetry
                telemetry.addData("Port range", String.format("%.01f in", portDistance));
                telemetry.addData("Starboard range", String.format("%.01f in",starboardDistance));
                telemetry.update();
            }

            while(averageDistance>6)
            {
                drive.followTrajectory(MediumCrawl);
                portDistance = sensorDistancePort.getDistance(DistanceUnit.INCH);
                starboardDistance = sensorDistanceStarboard.getDistance(DistanceUnit.INCH);
                averageDistance = (portDistance + starboardDistance) / 2;

                //Update Telemetry
                telemetry.addData("Port range", String.format("%.01f in", portDistance));
                telemetry.addData("Starboard range", String.format("%.01f in",starboardDistance));
                telemetry.update();
            }
            */
            //Get April Tag Telemetry

            //TODO Improve Efficiency

            boolean TagFound = false;
            while(!TagFound) {
                telemetryAprilTag();
                telemetry.update();
                for (AprilTagDetection detection : aprilTag.getDetections()) {
                    if (detection.metadata != null) {
                        if (detection.id > TeamPropPosition) {
                            drive.followTrajectorySequence(AprilTagSlideLeft);
                            break;
                        } else if (detection.id < TeamPropPosition) {
                            drive.followTrajectorySequence(AprilTagSlideRight);
                            break;
                        } else if (detection.id == TeamPropPosition) {
                            TagFound = true;
                            break;
                        }
                    }
                }
            }
            //Stop after you have found April Tag
            visionPortal.stopStreaming();

            if(TeamPropPosition == 4) {
                drive.followTrajectorySequence(LeftPark);
            } else if(TeamPropPosition == 5) {
                drive.followTrajectorySequence(CenterPark);
            } else if(TeamPropPosition == 6) {
                drive.followTrajectorySequence(RightPark);
            }

        }


        //Close at end of OpMode
        visionPortal.close();

    }  // end runOpMode()

    //Put Custom Methods Here
    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.8f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()


    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()


}  // end of class
