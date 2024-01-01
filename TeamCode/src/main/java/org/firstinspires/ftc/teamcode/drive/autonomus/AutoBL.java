package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.structure.Claw;

@Autonomous
public class AutoBL extends LinearOpMode {

        BlueWebCam camera = new BlueWebCam();
        SampleMecanumDrive drive;
        Claw sclaw;

        int caz = 3;

        //Gamepad Null;

        Vector2d parkVector = new Vector2d(58,61);

        Pose2d startPose = new Pose2d( 12, 62, Math.toRadians(-90));

        Vector2d startVector = new Vector2d( 12, 59);

        Vector2d centerSpike = new Vector2d(12,34);


        TrajectorySequence case_1,case_2,case_3, case_1_back, case_2_back, case_3_back;

        TrajectorySequence traj1,traj2,traj3;


        ElapsedTime reference_timer = new ElapsedTime();


        enum Paths{
            CenterSpike,
            PixelPlacement,
            BackToCenter,
            BackToStart,
            Park,
            Idle
        }

        private final FtcDashboard dashboard = FtcDashboard.getInstance();

        Paths paths = Paths.CenterSpike;

        @Override
        public void runOpMode() throws InterruptedException {

            TelemetryPacket packet = new TelemetryPacket();

            dashboard.setTelemetryTransmissionInterval(25);

            //sclaw = new Claw(hardwareMap, gamepad1);
            //sleep(1000);
            //sclaw.open = false;

            //camera.init(hardwareMap);


            drive = new SampleMecanumDrive(hardwareMap);

            drive.setPoseEstimate(startPose);


            traj1 = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(centerSpike)
                    .build();

            case_1 = drive.trajectorySequenceBuilder(traj1.end())
                    .turn(Math.toRadians(92))
                    .forward(2)
                    //.lineTo(case1)
                    .build();

            /*case_2 = drive.trajectorySequenceBuilder(traj1.end())
                    .lineTo(case2)
                    .build();*/

            case_3 = drive.trajectorySequenceBuilder(traj1.end())
                    .turn(Math.toRadians(-92))
                    .forward(2)
                    //.lineTo(case3)
                    .build();

            case_1_back = drive.trajectorySequenceBuilder(case_1.end())
                    .back(2)
                    .turn(Math.toRadians(-92))
                    .build();

            /*case_2_back = drive.trajectorySequenceBuilder(case_2.end())
                    .setReversed(true)
                    .lineTo(centerSpike)
                    .setReversed(false)
                    .build();*/

            case_3_back = drive.trajectorySequenceBuilder(case_3.end())
                    .back(2)
                    .turn(Math.toRadians(92))
                    .build();

            traj2 = drive.trajectorySequenceBuilder(case_1_back.end())
                    .setReversed(true)
                    .lineTo(startVector)
                    .setReversed(false)
                    .build();
            traj3 = drive.trajectorySequenceBuilder(traj2.end())
                    .turn(Math.toRadians(-92))
                    .setReversed(true)
                    .lineTo(parkVector)
                    .setReversed(false)
                    .build();


            /*while (!opModeIsActive()) {
                caz = camera.getAutoCase();
                packet.put("case", caz);
                dashboard.startCameraStream(camera.camera, 60);
                telemetry.addData("case", caz);
                dashboard.sendTelemetryPacket(packet);
                telemetry.update();

            }*/

            waitForStart();

            if (isStopRequested()) return;


            while (!isStopRequested() && opModeIsActive()) {

                switch (paths) {
                    case CenterSpike:
                        if (!drive.isBusy()) {
                            drive.followTrajectorySequenceAsync(traj1);
                            paths = Paths.PixelPlacement;
                        }
                        break;
                    case PixelPlacement:
                        if (!drive.isBusy()) {
                            if (caz == 3) {
                                drive.followTrajectorySequenceAsync(case_3);
                            } else {
                                drive.followTrajectorySequenceAsync(case_1);
                            }

                            paths = Paths.BackToCenter;
                        }

                        break;
                    case BackToCenter:
                        if (!drive.isBusy()) {
                            //sclaw.open = true;
                            sleep(1000);
                            //sclaw.open = false;

                            if (caz == 3) {
                                drive.followTrajectorySequenceAsync(case_3_back);
                            } else if (caz == 2) {
                                drive.followTrajectorySequenceAsync(case_2_back);
                            } else drive.followTrajectorySequenceAsync(case_1_back);

                            paths = Paths.BackToStart;
                        }
                        break;
                    case BackToStart:

                        if (!drive.isBusy()) {


                            drive.followTrajectorySequenceAsync(traj2);

                            paths = Paths.Park;

                        }

                        break;

                    case Park:

                        if (!drive.isBusy()) {
                            drive.followTrajectorySequenceAsync(traj3);
                            paths = Paths.Idle;
                        }
                    break;


                    case Idle:

                        break;
                }

                drive.update();
                telemetry.update();
            }

        }

}
