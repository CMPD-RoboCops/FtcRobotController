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
@Autonomous(name= "MJB Autonomous Mode",group = "drive")
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


    @Override
    public void runOpMode() throws InterruptedException
    {
        //Initiate TensorFlow
        initTfod();

        //Update Telemetry
        telemetry.addData(">", "Touch Play to start Autonomous");
        telemetry.update();

        //RoadRunner Initiation
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Setup Sensors
        sensorDistancePort = hardwareMap.get(DistanceSensor.class, "distancesensorport");
        sensorDistanceStarboard = hardwareMap.get(DistanceSensor.class, "distancesensorstarboard");

        colorPort = hardwareMap.get(ColorSensor.class, "colorsensorport");
        colorStarboard = hardwareMap.get(ColorSensor.class, "colorsensorstarboard");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistancePort;
        Rev2mDistanceSensor sensorTimeOfFlightStarboard = (Rev2mDistanceSensor) sensorDistanceStarboard;

        //Define Your Trajectories Here
        Trajectory RightSpikeTurn = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(16.2) //90 degree turn right
                .build();

        Trajectory LeftSpikeTurn = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(14.7) //90 degree turn left
                .build();

        Trajectory SideSpikeOvershoot = drive.trajectoryBuilder(new Pose2d())
                .forward(6) //90 degree turn left
                .build();

        Trajectory SideSpikeForward = drive.trajectoryBuilder(new Pose2d())
                .forward(18) // Tune Me
                .build();

        Trajectory CenterSpikeForward = drive.trajectoryBuilder(new Pose2d())
                .forward(24) // Tune Me
                .build();

        Trajectory SideSpikeLineForward = drive.trajectoryBuilder(new Pose2d())
                .forward(10) // Tune Me
                .build();

        Trajectory SpikeLineSeek = drive.trajectoryBuilder(new Pose2d())
                .forward(-1) // Tune Me
                .build();

        Trajectory LeftSpikeRecover = drive.trajectoryBuilder(new Pose2d())
                .forward(-8) //180 degree turn right
                .build();

        Trajectory CenterSpikeRecover = drive.trajectoryBuilder(new Pose2d())
                .forward(-6)
                .build();

        Trajectory BackSideDriveToBoard = drive.trajectoryBuilder(new Pose2d())
                .forward(60) // Tune Me
                .build();

        Trajectory BackwardCreep = drive.trajectoryBuilder(new Pose2d())
                .forward(-0.75) // Tune Me
                .build();

        //May not want to use this, may need to use smaller increments combined with sensor reads to avoid hitting poles
        Trajectory FrontSideDriveToBoard = drive.trajectoryBuilder(new Pose2d())
                .forward(100) // Tune Me
                .build();

        Trajectory Backup = drive.trajectoryBuilder(new Pose2d())
                .forward(-5) // Tune Me
                .build();

        Trajectory TurnAround = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(29.4) // Tune Me
                .build();

        //Control Structure Variables
        boolean lineFound = false;
        String TeamPropPosition = "";
        boolean SpikeLineFound = false;
        int DistanceToBoard = 1;


        //Very last thing before OpModeIsActive
        waitForStart();

        if (opModeIsActive())
        {
            telemetryTfod();

            while(tfod.getRecognitions().size()==0)
            {
                telemetry.addData("Image", tfod.getRecognitions().size());
            }

            //Team Prop Detection Control Structure
            if(!lineFound)
            {
                List<Recognition> currentRecognitions = tfod.getRecognitions();
                for (Recognition recognition : currentRecognitions)
                {
                    double CarX = (recognition.getLeft() + recognition.getRight()) / 2;
                    telemetry.addData("X Position", CarX);
                    if (CarX < 250)
                    {
                        TeamPropPosition = "Left";
                        telemetry.addData("Line", TeamPropPosition);
                        lineFound = true;
                    }
                    else if (CarX > 250 && CarX < 500)
                    {
                        TeamPropPosition = "Center";
                        telemetry.addData("Line", TeamPropPosition);
                        lineFound = true;
                    }
                    else if (CarX > 500)
                    {
                        TeamPropPosition = "Right";
                        telemetry.addData("Line", TeamPropPosition);
                        lineFound = true;
                    }
                    else
                    {
                        telemetry.addData("Line", "Unknown");
                        lineFound = false;
                    }
                }
            }

            // Push telemetry to the Driver Station and turn off streaming
            telemetry.update();
            //visionPortal.stopStreaming();


            //Go to Correct Spike Line
            if(TeamPropPosition.equals("Center"))
            {
                //Move up to SpikeLine, should overshoot a slight amount
                drive.followTrajectory(CenterSpikeForward);


                //Red Alliance Code, TODO Implement Blue Alliance Code
                while (SpikeLineFound == false)
                {
                    if (colorStarboard.red() > 400 || colorPort.red() > 400) {
                        SpikeLineFound = true;
                        //TODO Drop Pixel Code Here
                        drive.followTrajectory(CenterSpikeRecover);
                        drive.followTrajectory(RightSpikeTurn);
                        DistanceToBoard = 21;
                    } else {
                        drive.followTrajectory(BackwardCreep);
                    }
                }

            }

            if(TeamPropPosition.equals("Left"))
            {
                //Move up to SpikeLine, should overshoot a slight amount
                drive.followTrajectory(SideSpikeForward);
                drive.followTrajectory(LeftSpikeTurn);
                drive.followTrajectory(SideSpikeOvershoot);

                //Red Alliance Code, TODO Implement Blue Alliance Code
                while (SpikeLineFound == false)
                {
                    if (colorStarboard.red() > 400 || colorPort.red() > 400) {
                        SpikeLineFound = true;
                        drive.followTrajectory(LeftSpikeRecover);
                        drive.followTrajectory(TurnAround);
                        DistanceToBoard = 18;
                        //Drop Pixel Here
                    } else {
                        drive.followTrajectory(BackwardCreep);
                    }
                }
            }

            if(TeamPropPosition.equals("Right"))
            {
                //Move up to SpikeLine, should overshoot a slight amount
                drive.followTrajectory(SideSpikeForward);
                drive.followTrajectory(RightSpikeTurn);

                //Red Alliance Code, TODO Implement Blue Alliance Code
                while (SpikeLineFound == false)
                {
                    if (colorStarboard.red() > 600 || colorPort.red() > 600) {
                        SpikeLineFound = true;
                        DistanceToBoard = 12;
                        //Drop Pixel Here
                    } else {
                        drive.followTrajectory(BackwardCreep);
                    }
                }
                //TODO Recover so you don't run over and move your pixel
            }

            double portDistance = sensorDistancePort.getDistance(DistanceUnit.INCH);
            double starboardDistance = sensorDistanceStarboard.getDistance(DistanceUnit.INCH);
            double averageDistance = (portDistance + starboardDistance) / 2;

            Trajectory FastCrawl = drive.trajectoryBuilder(new Pose2d())
                    .forward(5)
                    .build();

            telemetry.addData("Port range", String.format("%.01f in", portDistance));
            telemetry.addData("Starboard range", String.format("%.01f in",starboardDistance));
            telemetry.update();

            while(averageDistance>14)
            {

                //TODO Remove Sleep
                sleep(1000);
                drive.followTrajectory(FastCrawl);
                portDistance = sensorDistancePort.getDistance(DistanceUnit.INCH);
                starboardDistance = sensorDistanceStarboard.getDistance(DistanceUnit.INCH);
                averageDistance = (portDistance + starboardDistance) / 2;

                //Update Telemetry
                telemetry.addData("Port range", String.format("%.01f in", portDistance));
                telemetry.addData("Starboard range", String.format("%.01f in",starboardDistance));
                telemetry.update();
            }

            //Get April Tag Telemetry
            //visionPortal.resumeStreaming();
            //telemetryAprilTag();

            //Stop after you have found April Tag
            visionPortal.stopStreaming();

            //






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
    /*
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
*/

}  // end of class
