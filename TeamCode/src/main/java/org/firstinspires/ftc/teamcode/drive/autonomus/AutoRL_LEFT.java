package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutoRL_LEFT extends LinearOpMode {

    RedWebCam camera = new RedWebCam();
    SampleMecanumDrive drive;
    //Claw sclaw;

    int caz = 2;

    //Gamepad Null;
    TrajectorySequence traj1;
    TrajectorySequence traj2;
    TrajectorySequence traj3;

    Pose2d startPose = new Pose2d(-35.2, -62, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        camera.init(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        Vector2d StartRL = new Vector2d( -35.2, -58.5);

        traj2 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-35.2,-12))
                .waitSeconds(1)
                .turn(Math.toRadians(-180))
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .lineTo(new Vector2d(58, -12))
                .setReversed(false)
                .build();

        traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-35.2,-36))
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .lineTo(new Vector2d(-35.2, -12))
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .lineTo(new Vector2d(58, -12))
                .setReversed(false)
                .build();

        traj3 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-35.2,-36))
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .lineTo(new Vector2d(-35.2, -12))
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .lineTo(new Vector2d(58, -12))
                .setReversed(false)
                .build();

        while(opModeInInit()){
            caz = camera.getAutoCase();
            telemetry.addData("[RL-LEFT] Case: ", caz);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;
        switch(caz){
            case 1:
                drive.followTrajectorySequenceAsync(traj1);
                break;
            case 2:
                drive.followTrajectorySequenceAsync(traj2);
                break;
            case 3:
                drive.followTrajectorySequenceAsync(traj3);
                break;
        }

        while (!isStopRequested() && opModeIsActive()) {

            drive.update();
        }

    }

}