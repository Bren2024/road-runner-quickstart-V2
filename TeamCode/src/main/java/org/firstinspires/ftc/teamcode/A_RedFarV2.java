package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="Red Far V2",group = "AAA")
public class A_RedFarV2 extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(-36.75, -62.5, Math.toRadians(0));//11.25-48(two tiles)

        drive.setPoseEstimate(startPose);

        TrajectorySequence leftTraj1 = drive.trajectorySequenceBuilder(startPose)
                //go to prop
                .lineToLinearHeading(new Pose2d(-46.0, -46, Math.toRadians(90))) //x:18-48(two tiles)-8 (other side of prop)
                .build();

        TrajectorySequence leftTraj2 = drive.trajectorySequenceBuilder(leftTraj1.end())
                //go to wall
                .lineToLinearHeading(new Pose2d(-42.0, -58.5, Math.toRadians(0))) //x:18-48(two tiles)-8 (other side of prop)
                .build();

        TrajectorySequence leftTraj3 = drive.trajectorySequenceBuilder(leftTraj2.end())
                //go past truss
                .lineToLinearHeading(new Pose2d(10.0, -58.5, Math.toRadians(0))) //x:18-48(two tiles)-8 (other side of prop)
                 //go to backdrop
                 .lineToLinearHeading(new Pose2d(50,-29, Math.toRadians(0)))
                .build();

        TrajectorySequence midTraj1 = drive.trajectorySequenceBuilder(startPose)
                //go to prop
                .lineToLinearHeading(new Pose2d(-38.0, -33, Math.toRadians(90))) //x:18-48(two tiles)-8 (other side of prop)
                .build();

        TrajectorySequence midTraj2 = drive.trajectorySequenceBuilder(midTraj1.end())
                //go to wall
                .lineToLinearHeading(new Pose2d(-42.0, -58.5, Math.toRadians(0))) //x:18-48(two tiles)-8 (other side of prop)
                //go past truss
                .lineToLinearHeading(new Pose2d(10.0, -58.5, Math.toRadians(0))) //x:18-48(two tiles)-8 (other side of prop)
                //go to backdrop
                .lineToLinearHeading(new Pose2d(50,-35, Math.toRadians(0)))
                .build();

        TrajectorySequence rightTraj1 = drive.trajectorySequenceBuilder(startPose)
                //go to prop
                .lineToLinearHeading(new Pose2d(-35, -37, Math.toRadians(0))) //x:18-48(two tiles)-8 (other side of prop)
                .build();

        TrajectorySequence rightTraj2 = drive.trajectorySequenceBuilder(rightTraj1.end())
                //go to wall
                .lineToLinearHeading(new Pose2d(-36.75, -37, Math.toRadians(0))) //x:18-48(two tiles)-8 (other side of prop)
                .lineToLinearHeading(new Pose2d(-36.75, -59, Math.toRadians(0))) //x:18-48(two tiles)-8 (other side of prop)
                //go past truss
                .lineToLinearHeading(new Pose2d(10.0, -59, Math.toRadians(0))) //x:18-48(two tiles)-8 (other side of prop)
                //go to backdrop
                //.lineToLinearHeading(new Pose2d(50,-42, Math.toRadians(0)))
                .build();

        TrajectorySequence moveToBackboard = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(0, -58.5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50, -35), Math.toRadians(30))
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

        if (nPropPos == goggles2.PROP_RIGHT) {
            drive.followTrajectorySequence(rightTraj1);
            drive.followTrajectory(buildCorrectionTrajectory(rightTraj1.end(), 10, 10));
            piranhatail.autonFlickPixel(this,2200,100);
            drive.followTrajectorySequence(rightTraj2);
            drive.followTrajectory(buildCorrectionTrajectory(rightTraj2.end(), 10, 10));
        }
        else if (nPropPos == goggles2.PROP_MID) {
            drive.followTrajectorySequence(midTraj1);
            drive.followTrajectory(buildCorrectionTrajectory(midTraj1.end(), 10, 10));
            piranhatail.autonFlickPixel(this,2200,100);
            drive.followTrajectorySequence(midTraj2);
            drive.followTrajectory(buildCorrectionTrajectory(midTraj2.end(), 10, 10));
        }
        else {  //LEFT
            drive.followTrajectorySequence(leftTraj1);
            drive.followTrajectory(buildCorrectionTrajectory(leftTraj1.end(), 10, 10));
            piranhatail.autonFlickPixel(this,2200,100);
            drive.followTrajectorySequence(leftTraj2);
            drive.followTrajectory(buildCorrectionTrajectory(leftTraj2.end(), 10, 10));
            drive.followTrajectorySequence(leftTraj3);
            drive.followTrajectory(buildCorrectionTrajectory(leftTraj3.end(), 10, 10));
        }

//        Trajectory moveToPark = drive.trajectoryBuilder(chosenTraj.end())
//             .strafeTo(new Vector2d(48, -60))
//                .build(); // traj instead of trajSeq for simplicity as this is building during autonomous
        freezeray.autonShootPixel3(this,0.472,0.524,3000,10000);

//        drive.followTrajectory(moveToPark);
        //TODO: COMMENT OUT BELOW WHEN DONE!!
        TrajectorySequence returnBack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                //go to front of truss
                .lineToLinearHeading(new Pose2d(10.0, -58.5, Math.toRadians(0))) //x:18-48(two tiles)-8 (other side of prop)
                //go to back of truss

                .lineToLinearHeading(new Pose2d(-42.0, -59.5, Math.toRadians(0))) //x:18-48(two tiles)-8 (other side of prop)
                //go past truss
                .lineToLinearHeading(startPose)
                .build();
        drive.followTrajectorySequence(returnBack);
    }

    private Trajectory buildCorrectionTrajectory(Pose2d pose) {
        Trajectory correction = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-35, -37, Math.toRadians(0))) //x:18-48(two tiles)-8 (other side of prop)
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
