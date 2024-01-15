package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//New Imports
import com.acmerobotics.roadrunner.trajectory.Trajectory;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                //.strafeRight(18) //90 degree turn right
                .strafeLeft(35) //180 degree turn left
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(10)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj1.end())
                .forward(8)
                .build();

        //Pose2d myPose = new Pose2d(10, -5, Math.toRadians(90));

        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(0, 2), -165)
                .build();

        //drive.followTrajectory(traj2);
        //drive.followTrajectory(traj1);
        //drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);

        while (!isStopRequested()) {
            /*drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();*/

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();


        }
    }
}
