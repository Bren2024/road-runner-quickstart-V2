package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(group = "drive")
public class A_RedNearBackdropV1 extends LinearOpMode {

    private SampleSwerveDrive drive;
    private Goggles2V3AS goggles2 = new Goggles2V3AS();
    private PiranhaDogV4AS piranhadog = new PiranhaDogV4AS();
    private FreezeRay4BarV1AS freezeray = new FreezeRay4BarV1AS();
    private String gstrClassName=this.getClass().getSimpleName();

    @Override
    public void runOpMode() {
        int nTagToFind=-1;
        int nPropPos=0;
        int chosenTraj;

        drive = new SampleSwerveDrive(hardwareMap);

        goggles2.initialize(this,goggles2.RED_CAM);//Red is 'Webcam 1

        piranhadog.initialize(this);
        freezeray.initialize(this);

        Pose2d startPose = new Pose2d(14.75, -62.5, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(12, -36))
                .addTemporalMarker(() -> {
                    piranhadog.autonSpitPixel(this, 750, 1000);
                })
                .waitSeconds(2)
                .strafeTo(new Vector2d(50, -30),
                        SampleSwerveDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleSwerveDrive.getAccelerationConstraint(15))
                .build();

        TrajectorySequence midTraj = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(26, -24))
                .addTemporalMarker(() -> {
                    piranhadog.autonSpitPixel(this, 750, 1000);
                })
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(50, -35, Math.toRadians(0)),
                        SampleSwerveDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleSwerveDrive.getAccelerationConstraint(15))
                .build();

        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(33,-36, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    piranhadog.autonSpitPixel(this, 750, 1000);
                })
                .waitSeconds(2)
                .strafeTo(new Vector2d(50, -42),
                        SampleSwerveDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleSwerveDrive.getAccelerationConstraint(15))
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

        if (nPropPos == goggles2.PROP_LEFT) {
            drive.followTrajectorySequence(leftTraj);
            drive.followTrajectory(buildCorrectionTrajectory(leftTraj.end(), 5, 5));
        }
        else if (nPropPos == goggles2.PROP_MID) {
            drive.followTrajectorySequence(midTraj);
            drive.followTrajectory(buildCorrectionTrajectory(midTraj.end(), 5, 5));
        }
        else {
            drive.followTrajectorySequence(rightTraj);
            drive.followTrajectory(buildCorrectionTrajectory(rightTraj.end(), 5, 5));
        }
//        Trajectory moveToPark = drive.trajectoryBuilder(chosenTraj.end())
//                .strafeTo(new Vector2d(48, -60))
//                .build(); // traj instead of trajSeq for simplicity as this is building during autonomous

//        freezeray.autonShootPixel2(this,freezeray.RAY_POS_UNHOLSTER,0.472,0.528,0.59,2000,7000);
        freezeray.autonShootPixel3(this,0.472,0.524,3000,10000);
//        drive.followTrajectory(moveToPark);
        Trajectory returnBack = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(startPose)
                .build();
        drive.followTrajectory(returnBack);
    }

    private Trajectory buildCorrectionTrajectory(Pose2d pose) {
        Trajectory correction = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(pose)
                .build();
        return correction;
    }
    private Trajectory buildCorrectionTrajectory(Pose2d pose, double maxVel, double maxAccel) {
        Trajectory correction = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(pose,
                        SampleSwerveDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleSwerveDrive.getAccelerationConstraint(maxAccel))
                .build();
        return correction;
    }
}
