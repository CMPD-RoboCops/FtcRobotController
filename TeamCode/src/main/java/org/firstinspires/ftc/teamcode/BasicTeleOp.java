package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import org.json.JSONException;
import org.json.JSONObject;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "Test Modes")
public class BasicTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        String robotName = "Unknown Robot";

        // Get the configuration file from the Control Hub's internal storage
        // File is stored in the /FIRST/settings folder
        File configFile = AppUtil.getInstance().getSettingsFile("robot_config.json");

        // Read and parse the JSON file
        try (FileReader reader = new FileReader(configFile)) {
            char[] buffer = new char[(int) configFile.length()];
            reader.read(buffer);
            String jsonString = new String(buffer);
            JSONObject jsonObject = new JSONObject(jsonString);
            robotName = jsonObject.getString("robotName");
        } catch (IOException e) {
            telemetry.addData("Error", "Cannot read config file: " + e.getMessage());
        } catch (Exception e) {
            telemetry.addData("Error", "Parsing error: " + e.getMessage());
        }
        telemetry.addData("Robot Name", robotName);
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
