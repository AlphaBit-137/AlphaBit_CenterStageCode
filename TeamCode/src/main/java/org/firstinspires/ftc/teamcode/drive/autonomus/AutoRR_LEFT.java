package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutoRR_LEFT extends LinearOpMode {

    RedWebCam camera = new RedWebCam();
    SampleMecanumDrive drive;
    //Claw sclaw;

    int caz = 2;

    //Gamepad Null;
    TrajectorySequence traj1;
    TrajectorySequence traj2;
    TrajectorySequence traj3;

    Pose2d startPose = new Pose2d(12, -62, Math.toRadians(90));
    Vector2d StartRR = new Vector2d( 12, -58);

    @Override
    public void runOpMode() throws InterruptedException {
        camera.init(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        traj2 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12,-36))
                .lineTo(StartRR)
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(35,-58.5))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(35, -12))
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(58, -12))
                .setReversed(false)
                .build();

        traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12,-36))
                .turn(Math.toRadians(90))
                .turn(Math.toRadians(-90))
                .lineTo(StartRR)
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(35,-57))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(35, -12))
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(58, -12))
                .setReversed(false)
                .build();

        traj3 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12,-36))
                .turn(Math.toRadians(-90))
                .turn(Math.toRadians(90))
                .lineTo(StartRR)
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(35,-58.5))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(35, -12))
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(58, -12))
                .setReversed(false)
                .build();

        while(opModeInInit()){
            caz = camera.getAutoCase();
            telemetry.addData("[RR-LEFT] Case: ", caz);
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