package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name="Blue Near V3 Shear Force",group = "AAA")
public class A_BlueNearV3ShearForce extends LinearOpMode {

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

        goggles2.initialize(this,goggles2.BLUE_CAM);//Red is 'Webcam 1

        piranhadog.initialize(this);
        freezeray.initialize(this);
        piranhatail.initialize(this,piranhatail.TAIL_INIT_AUTON);

        Pose2d startPose = new Pose2d(11.25, 62.5, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence leftTraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(15,44.5, Math.toRadians(-60)))
                .build();

        TrajectorySequence leftTraj2 = drive.trajectorySequenceBuilder(leftTraj1.end())
                .lineToLinearHeading(new Pose2d(24,55, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    freezeray.autonRaiseWeaponHeight(this,1650);
                })
                .lineToLinearHeading(new Pose2d(51.5,45.5, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    freezeray.autonRaiseWeaponHeight(this,1650); //1575
                    freezeray.autonAimWeapon(this,.470d,0.530d); //left .472 right 524
                })
                .waitSeconds(1)
                //release pixel
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> { // Can call other parts of the robot
                    freezeray.autonShoot(this);
                })
                .build();

        TrajectorySequence midTraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(14, 36.5, Math.toRadians(-90)))
                .build();

        TrajectorySequence midTraj2 = drive.trajectorySequenceBuilder(midTraj1.end())
                .strafeTo(new Vector2d(18, 37))
                .addTemporalMarker(() -> {
                    freezeray.autonRaiseWeaponHeight(this,1600); //1575
                })
                .lineToLinearHeading(new Pose2d(51,38, Math.toRadians(0)))
                .build();

        TrajectorySequence midTraj3 = drive.trajectorySequenceBuilder(midTraj2.end())
                .addTemporalMarker(() -> {
                    freezeray.autonRaiseWeaponHeight(this,1600); //1575
                    freezeray.autonAimWeapon(this,.470d,0.530d); //left .472 right 524
                })
                //release pixel
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> { // Can call other parts of the robot
                    freezeray.autonShoot(this);
                })
                .waitSeconds(1)
                .back(6)
                .build();

        TrajectorySequence rightTraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12,38, Math.toRadians(-179)))
                .addTemporalMarker(() -> { // Can call other parts of the robot
                    piranhatail.autonFlickPixel(this,2200,100);
                })
                .waitSeconds(2.2) //let pixel drop on floor
                .addTemporalMarker(() -> {
                    freezeray.autonRaiseWeaponHeight(this,1600);
                })
                .lineToLinearHeading(new Pose2d(52,30, Math.toRadians(0)))
                .build();

        TrajectorySequence rightTraj2 = drive.trajectorySequenceBuilder(rightTraj1.end())
                //extend bipod
                .addTemporalMarker(() -> {
                    freezeray.autonRaiseWeaponHeight(this,1600); //1575
                    freezeray.autonAimWeapon(this,.470d,0.530d); //left .472 right 524
                })
                //release pixel
                .addTemporalMarker(.5, () -> { // Can call other parts of the robot
                    freezeray.autonShoot(this);
                })
                .waitSeconds(1)
                .back(6)
                .build();

        telemetry.addData(gstrClassName, "Initialized");
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;

        long timestamp = System.currentTimeMillis();

        nPropPos=goggles2.findProp(this,8000);

        telemetry.addData(gstrClassName, "Prop position:%d",nPropPos);
        telemetry.update();

        while (System.currentTimeMillis() < timestamp + 10000);

        if (nPropPos == goggles2.PROP_NONE)
            nPropPos = goggles2.PROP_LEFT;

        if (nPropPos == goggles2.PROP_LEFT) {
            drive.followTrajectorySequence(leftTraj1);
            drive.followTrajectory(buildCorrectionTraj(leftTraj1.end(), 10, 10));
            piranhatail.autonFlickPixel(this,2200,100);
            drive.followTrajectorySequence(leftTraj2);
            drive.followTrajectory(buildCorrectionTraj(leftTraj2.end(), 10, 10));
            freezeray.autonMakeWeaponSafe(this);
        }
        else if (nPropPos == goggles2.PROP_MID) {
            drive.followTrajectorySequence(midTraj1);
            drive.followTrajectory(buildCorrectionTraj(midTraj1.end(), 10, 10));
            piranhatail.autonFlickPixel(this,2200,100);
            drive.followTrajectorySequence(midTraj2);
            drive.followTrajectory(buildCorrectionTraj(midTraj2.end(), 10, 10));
            drive.followTrajectorySequence(midTraj3);
            freezeray.autonMakeWeaponSafe(this);
        }
        else {
            drive.followTrajectorySequence(rightTraj1);
            drive.followTrajectory(buildCorrectionTraj(rightTraj1.end(), 10, 10)); // Use extra correction b/c very inaccurate
            drive.followTrajectorySequence(rightTraj2);
            freezeray.autonMakeWeaponSafe(this);
        }

        Trajectory moveToPark = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(48, 58))
                .build(); // traj instead of trajSeq for simplicity as this is building during autonomous
        drive.followTrajectory(moveToPark);

        //TODO: COMMENT OUT BELOW WHEN DONE!!
//        Trajectory returnBack = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .lineToLinearHeading(startPose)
//                .build();
//        drive.followTrajectory(returnBack);
    }

    /**
     * Creates a trajectory that strafes from current estimated position to target position
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
