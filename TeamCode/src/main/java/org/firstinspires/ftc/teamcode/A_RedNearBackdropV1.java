package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class A_RedNearBackdropV1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleSwerveDrive drive = new SampleSwerveDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, -63, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(18,-46.5, Math.toRadians(45)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(12, -30, Math.toRadians(0)), Math.toRadians(90))
                .waitSeconds(2)
                .strafeTo(new Vector2d(48, -30))
                .build()
                ;

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(leftTraj);
    }
}
