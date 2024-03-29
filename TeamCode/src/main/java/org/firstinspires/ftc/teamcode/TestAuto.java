package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class TestAuto extends LinearOpMode {
    private PiranhaDogV4AS piranhadog = new PiranhaDogV4AS();
    @Override
    public void runOpMode() {
        SampleSwerveDrive drive = new SampleSwerveDrive(hardwareMap);
        piranhadog.initialize(this);

        Pose2d startPose = new Pose2d(14.75, -62.5, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(24, -48))
                .build()
                ;

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(traj);
    }
}
