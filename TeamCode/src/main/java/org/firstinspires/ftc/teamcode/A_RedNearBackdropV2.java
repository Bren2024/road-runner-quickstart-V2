package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.opencv.core.Mat;


@Autonomous(group = "drive")
public class A_RedNearBackdropV2 extends LinearOpMode {

    private Goggles2V3AS goggles2 = new Goggles2V3AS();
    private PiranhaDogV4AS piranhadog = new PiranhaDogV4AS();
    private FreezeRayAS freezeray = new FreezeRayAS();
    private RocketAS rocket = new RocketAS();
    private String gstrClassName=this.getClass().getSimpleName();

    @Override
    public void runOpMode() {
        int nTagToFind=-1;
        int nPropPos=0;
        TrajectorySequence chosenTraj;

        SampleSwerveDrive drive = new SampleSwerveDrive(hardwareMap);
        goggles2.initialize(this,goggles2.RED_CAM);//Red is 'Webcam 1

        piranhadog.initialize(this);
        freezeray.initialize(this);
        rocket.initialize(this);


        Pose2d startPose = new Pose2d(15, -63, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(12, -30))
//                .addDisplacementMarker(() -> {
//                    piranhadog.autonSpitPixel(this, 750, 1000);
//                })
                .waitSeconds(2)
                .strafeTo(new Vector2d(52, -28))
                .build();

        TrajectorySequence midTraj1 = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(12, -36))
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence midTraj2 = drive.trajectorySequenceBuilder(midTraj1.end())
//                .addDisplacementMarker(() -> {
//                    piranhadog.autonSpitPixel(this, 750, 1000);
//                })
                .waitSeconds(2)
                .strafeTo(new Vector2d(36, -36))
                .turn(Math.toRadians(90))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(52, -36, Math.toRadians(0)))
                .build();

        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(36,-30, Math.toRadians(0)))
//                .addDisplacementMarker(() -> {
//                    piranhadog.autonSpitPixel(this, 750, 1000);
//                })
                .waitSeconds(2)
                .strafeTo(new Vector2d(52, -42))
                .build();

        TrajectorySequence V2Traj = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(48, -32, Math.toRadians(0)))
                .waitSeconds(2)
                .splineToSplineHeading(new Pose2d(12,-32, Math.toRadians(90)), Math.toRadians(90))
                .build();

        telemetry.addData(gstrClassName, "Initialized");
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;

        nPropPos=goggles2.findProp(this,5000);

        telemetry.addData(gstrClassName, "Prop position:%d",nPropPos);
        telemetry.update();

        if (nPropPos == goggles2.PROP_NONE)
            nPropPos = goggles2.PROP_RIGHT;

        if (nPropPos == goggles2.PROP_LEFT)
            chosenTraj = leftTraj;
        else if (nPropPos == goggles2.PROP_MID)
            chosenTraj = midTraj1;
        else
            chosenTraj = rightTraj;

//        Trajectory moveToPark = drive.trajectoryBuilder(chosenTraj.end())
//                .strafeTo(new Vector2d(48, -60))
//                .build(); // traj instead of trajSeq for simplicity as this is building during autonomous

        drive.followTrajectorySequence(chosenTraj);

        Trajectory correctSpit = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(-90)),
                        SampleSwerveDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleSwerveDrive.getAccelerationConstraint(5))
                .build();
        drive.followTrajectory(correctSpit);
        drive.followTrajectorySequence(midTraj2);
        Trajectory correctForBoard = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(52, -36),
                        SampleSwerveDrive.getVelocityConstraint(2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleSwerveDrive.getAccelerationConstraint(2))
                .build();
        drive.followTrajectory(correctForBoard);
        drive.getPoseEstimate();
        freezeray.autonShootPixel2(this,freezeray.RAY_POS_UNHOLSTER,0.472,0.528,0.59,2000,7000);
//        drive.followTrajectory(moveToPark);
    }
}
