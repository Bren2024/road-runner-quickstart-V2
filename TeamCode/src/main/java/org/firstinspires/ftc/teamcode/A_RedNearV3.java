package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="Red Near V3",group = "AAA")
public class A_RedNearV3 extends LinearOpMode {

    private SampleSwerveDrive drive;
    private Goggles2V3AS goggles2 = new Goggles2V3AS();
    private PiranhaDogV4AS piranhadog = new PiranhaDogV4AS();
    private PiranhaTailAS piranhatail = new PiranhaTailAS();
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
        piranhatail.initialize(this,piranhatail.TAIL_INIT_AUTON);

        Pose2d startPose = new Pose2d(11.25, -62.5, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(11,-36, Math.toRadians(180)))
                .addTemporalMarker(() -> { // Can call other parts of the robot
                    piranhatail.autonFlickPixel(this,2200,100);
                })
                .waitSeconds(2) //let pixel drop on floor
                .lineToLinearHeading(new Pose2d(51,-29, Math.toRadians(0)))
                .build();

        TrajectorySequence midTraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(18, -34, Math.toRadians(90)))
                .build();

        TrajectorySequence midTraj2 = drive.trajectorySequenceBuilder(midTraj1.end())
                .strafeTo(new Vector2d(18, -37))
                .lineToLinearHeading(new Pose2d(50,-35, Math.toRadians(0)))
                .build();

        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(19,-46, Math.toRadians(60)))
                .addTemporalMarker(() -> { // Can call other parts of the robot
                    piranhatail.autonFlickPixel(this,2200,100);
                })
                .waitSeconds(2.5) //let pixel drop on floor
                .lineToLinearHeading(new Pose2d(24,-55, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(50,-42, Math.toRadians(0)))
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
            drive.followTrajectorySequence(midTraj1);
            drive.followTrajectory(buildCorrectionTrajectory(midTraj1.end(), 10, 10));
            piranhatail.autonFlickPixel(this,2200,100);
            drive.followTrajectorySequence(midTraj2);
            drive.followTrajectory(buildCorrectionTrajectory(midTraj2.end(), 10, 10));
        }
        else {
            drive.followTrajectorySequence(rightTraj);
            drive.followTrajectory(buildCorrectionTrajectory(rightTraj.end(), 5, 5));
        }

//        Trajectory moveToPark = drive.trajectoryBuilder(chosenTraj.end())
//             .strafeTo(new Vector2d(48, -60))
//                .build(); // traj instead of trajSeq for simplicity as this is building during autonomous
        //freezeray.autonShootPixel2(this,freezeray.RAY_POS_UNHOLSTER,0.472,0.528,0.59,2000,7000);
        freezeray.autonShootPixel3(this,0.472,0.524,3000,10000);

//        drive.followTrajectory(moveToPark);
        //TODO: COMMENT OUT BELOW WHEN DONE!!
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
