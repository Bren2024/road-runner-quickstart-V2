package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="Red Far V4",group = "AAA")
public class A_RedFarV4 extends LinearOpMode {

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

        //////////////LEFT////////////////////////
        TrajectorySequence leftTraj1 = drive.trajectorySequenceBuilder(startPose)
                //go to prop
                .lineToLinearHeading(new Pose2d(-44, -43, Math.toRadians(90)))
                .build();

        TrajectorySequence leftTraj2 = drive.trajectorySequenceBuilder(leftTraj1.end())
                //go to wall
                .lineToLinearHeading(new Pose2d(-42, -58, Math.toRadians(0)))
                //go past truss
                .splineToLinearHeading(new Pose2d(10, -58, 0), Math.toRadians(0))
                //raise 4bar
                .addTemporalMarker(() -> {
                    freezeray.autonRaiseWeaponHeight(this,1200);
                })
                //go to backdrop
                .splineToLinearHeading(new Pose2d(51,-30, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence leftTraj3 = drive.trajectorySequenceBuilder(leftTraj2.end())
                //extend bipod
                .addTemporalMarker(() -> {
                    freezeray.autonRaiseWeaponHeight(this,1200);
                    freezeray.autonAimWeapon(this,.470d,0.530d); //left .472 right 524
                })
                //release pixel
                .addTemporalMarker(.75, () -> { // Can call other parts of the robot
                    freezeray.autonShoot(this);
                })
                .waitSeconds(1.5)
                //back away
                .back(6)
                .build();

        ////////////////////////MID/////////////////////
        TrajectorySequence midTraj1 = drive.trajectorySequenceBuilder(startPose)
                //go to prop
                .lineToLinearHeading(new Pose2d(-39.5, -34, Math.toRadians(90)))
                .build();

        TrajectorySequence midTraj2 = drive.trajectorySequenceBuilder(midTraj1.end())
                //go to wall
                .lineToLinearHeading(new Pose2d(-42, -58, Math.toRadians(0)))
                //go past truss
                .splineToLinearHeading(new Pose2d(0, -58, Math.toRadians(0)), Math.toRadians(0))
                //raise 4bar
                .addTemporalMarker(() -> {
                    freezeray.autonRaiseWeaponHeight(this,1200);
                })
                //go to backdrop
                .splineToLinearHeading(new Pose2d(50,-35, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence midTraj3 = drive.trajectorySequenceBuilder(midTraj2.end())
                //extend bipod
                .addTemporalMarker(() -> {
                    freezeray.autonRaiseWeaponHeight(this,1200);
                    freezeray.autonAimWeapon(this,.470d,0.530d); //left .472 right 524
                })
                //release pixel
                .addTemporalMarker(.75, () -> { // Can call other parts of the robot
                    freezeray.autonShoot(this);
                })
                .waitSeconds(1.5)
                //back away
                .back(6)
                .build();

        /////////////////RIGHT//////////////
        TrajectorySequence rightTraj1 = drive.trajectorySequenceBuilder(startPose)
                //go to prop
                .lineToLinearHeading(new Pose2d(-45, -37, Math.toRadians(0))) //-40
                //extend tail halfway
                .addTemporalMarker(1.5, () -> { // Can call other parts of the robot
                    piranhatail.autonSetFlickPixel(this, PiranhaTailAS.TAIL_HFLICK);
                })
                // align for tail drop
                .lineToLinearHeading(new Pose2d(-35, -37, Math.toRadians(0))) //38.5, 34
                .build();

        TrajectorySequence rightTraj2 = drive.trajectorySequenceBuilder(rightTraj1.end())
                //drop pixel
                .addTemporalMarker(() -> { // Can call other parts of the robot
                    piranhatail.autonSetFlickPixel(this, PiranhaTailAS.TAIL_FLICK);
                    sleep(1000);
                    piranhatail.autonSetFlickPixel(this, PiranhaTailAS.TAIL_HFLICK);
                })
                .waitSeconds(1)
                //go back out
                .lineToLinearHeading(new Pose2d(-44, -32, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    //store tail while moving away
                    piranhatail.autonSetFlickPixel(this, PiranhaTailAS.TAIL_BETWEEN_LEGS);
                })
                .lineToLinearHeading(new Pose2d(-42, -58.5, Math.toRadians(0))) //y=-58
                //go past truss
                //.lineToLinearHeading(new Pose2d(10, 58.5, Math.toRadians(0)))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(0, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    //raise 4bar
                    freezeray.autonRaiseWeaponHeight(this,1200);
                })
                //go to backdrop
                .splineToLinearHeading(new Pose2d(51, -42, Math.toRadians(0)), Math.toRadians(0))
                //.lineToLinearHeading(new Pose2d(48, -43.5, Math.toRadians(0)))
                .build();

        TrajectorySequence rightTraj3 = drive.trajectorySequenceBuilder(rightTraj2.end())
                //extend bipod
                .addTemporalMarker(() -> {
                    freezeray.autonRaiseWeaponHeight(this,1200);
                    freezeray.autonAimWeapon(this,.470d,0.530d); //left .472 right 524
                })
                //release pixel
                .addTemporalMarker(.75, () -> {
                    freezeray.autonShoot(this);
                })
                .waitSeconds(1.5)
                //back away
                .back(6)
                .build();


        telemetry.addData(gstrClassName, "Initialized");
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;

        nPropPos=goggles2.findProp(this,5000);

        telemetry.addData(gstrClassName, "Prop position:%d",nPropPos);
        telemetry.update();

        if (nPropPos == goggles2.PROP_NONE)
            nPropPos = goggles2.PROP_LEFT;

        if (nPropPos == goggles2.PROP_RIGHT) {
            //get to spike mark and drop pixel
            drive.followTrajectorySequence(rightTraj1);
            drive.followTrajectory(buildCorrectionTraj(rightTraj1.end(), 10, 10));
            drive.followTrajectorySequence(rightTraj2);
            drive.followTrajectory(buildCorrectionTraj(rightTraj2.end(), 10, 10));
            drive.followTrajectorySequence(rightTraj3);
            freezeray.autonMakeWeaponSafe(this);
        }
        else if (nPropPos == goggles2.PROP_MID) {
            //goto spike mark
            drive.followTrajectorySequence(midTraj1);
            drive.followTrajectory(buildCorrectionTraj(midTraj1.end(), 10, 10));
            piranhatail.autonFlickPixel(this,2200,100);
            drive.followTrajectorySequence(midTraj2);
            drive.followTrajectory(buildCorrectionTraj(midTraj2.end(), 10, 10));
            drive.followTrajectorySequence(midTraj3);
            freezeray.autonMakeWeaponSafe(this);

        }
        else {  //LEFT
            //go to spike mark
            drive.followTrajectorySequence(leftTraj1);
            drive.followTrajectory(buildCorrectionTraj(leftTraj1.end(), 10, 10));
            piranhatail.autonFlickPixel(this,2200,100);
            drive.followTrajectorySequence(leftTraj2);
            drive.followTrajectory(buildCorrectionTraj(leftTraj2.end(), 10, 10));
            drive.followTrajectorySequence(leftTraj3);
            freezeray.autonMakeWeaponSafe(this);
        }

        Trajectory moveToPark = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(48, -12))
                .build(); // traj instead of trajSeq for simplicity as this is building during autonomous

        drive.followTrajectory(moveToPark);



        //TODO: COMMENT OUT BELOW WHEN DONE!!
//        TrajectorySequence returnBack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                //go to front of truss
//                .lineToLinearHeading(new Pose2d(10.0, -58.5, Math.toRadians(0))) //x:18-48(two tiles)-8 (other side of prop)
//                //go to back of truss
//
//                .lineToLinearHeading(new Pose2d(-42.0, -59.5, Math.toRadians(0))) //x:18-48(two tiles)-8 (other side of prop)
//                //go past truss
//                .lineToLinearHeading(startPose)
//                .build();
//        drive.followTrajectorySequence(returnBack);
    }

    /**
     * Creates a trajectory that strafes from current estimated pose to target pose
     * Note: Doesn't
     * @param pose
     * @param maxVel
     * @param maxAccel
     * @return
     */
    private Trajectory buildCorrectionTraj(Pose2d pose, double maxVel, double maxAccel) {
        Trajectory correction = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(pose,
                        SampleSwerveDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleSwerveDrive.getAccelerationConstraint(maxAccel))
                .build();
        return correction;
    }

    /**
     * Creates a trajectory that turns to the correct heading, then strafes to the correct position
     * @param pose
     * @param maxVel
     * @param maxAccel
     * @return Built trajectory
     */
    private TrajectorySequence buildCorrectionTraj2(Pose2d pose, double maxVel, double maxAccel) {
        TrajectorySequence correction = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                // Turn to correct
                .turn((pose.getHeading()-drive.getPoseEstimate().getHeading()+3*Math.PI)%(2*Math.PI)-Math.PI) // Clamp between -π & π
                // Strafe to correct
                .lineToLinearHeading(pose,
                        SampleSwerveDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleSwerveDrive.getAccelerationConstraint(maxAccel))
                .build();
        return correction;
    }
}
