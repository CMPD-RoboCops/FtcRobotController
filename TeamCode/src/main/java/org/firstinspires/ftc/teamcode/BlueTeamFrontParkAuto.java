package org.firstinspires.ftc.teamcode;

//NOTE: Do not use this to make other changes for a more advanced autonomous. This is meant to be a very simple parking.
//
//
//
//
//
//
//
//
//

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(name= "Blue Front Park Only Auto",group = "drive")
public class BlueTeamFrontParkAuto extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        //NOTE: Do not use this to make other changes for a more advanced autonomous. This is meant to be a very simple parking.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Trajectory Forward = drive.trajectoryBuilder(new Pose2d())
                .forward(59)
                .build();

        Trajectory Backward = drive.trajectoryBuilder(new Pose2d())
                .back(102)
                .build();

        Trajectory StrafeRight = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .build();

        waitForStart();

        if (opModeIsActive())
        {
            drive.followTrajectory(Forward);
            drive.turn(Math.toRadians(-100));
            drive.followTrajectory(Backward);
            drive.followTrajectory(StrafeRight);
        }
    }
}
