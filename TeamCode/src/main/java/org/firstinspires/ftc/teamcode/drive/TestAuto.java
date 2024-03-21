package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleSwerveDrive drive = new SampleSwerveDrive(hardwareMap);

        Pose2d startPose = new Pose2d(36, -63, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(9, -36), Math.toRadians(150))
                .waitSeconds(3)
                .lineToSplineHeading(new Pose2d(36, -36, Math.toRadians(0)))
                .build()
        ;

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(myTrajectory);
    }
}
