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
@Autonomous(group = "drive")
public class A_ExperimentRedNear extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(14.75, -62.5, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(2, () -> {
                    piranhadog.setTeethPwr(0.45);
                })
                .addTemporalMarker(2.5, () -> {
                    piranhadog.setTeethPwr(0);
                })
                .splineToConstantHeading(new Vector2d(14.75, -62), Math.toRadians(90)) // Start strafing left
                .splineToConstantHeading(new Vector2d(12, -32), Math.toRadians(90)) // Center on tile
                .splineToConstantHeading(new Vector2d(22, -23), Math.toRadians(0)) // Gradually turn wheels towards board
                .splineToConstantHeading(new Vector2d(50, -30), Math.toRadians(0)) // Line up at board
                .build();

        TrajectorySequence midTraj = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(2.7, () -> {
                    piranhadog.setTeethPwr(0.45);
                })
                .addTemporalMarker(3.1, () -> {
                    piranhadog.setTeethPwr(0);
                })
                .splineToConstantHeading(new Vector2d(14.75, -62), Math.toRadians(90)) // Start strafing left
                .splineToConstantHeading(new Vector2d(12, -32), Math.toRadians(90)) // Center on tile
                .splineToConstantHeading(new Vector2d(22, -23), Math.toRadians(0)) // Gradually turn wheels towards board
                .splineToConstantHeading(new Vector2d(50, -35), Math.toRadians(0)) // Line up at board
                .build();

        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(
                        SampleSwerveDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleSwerveDrive.getAccelerationConstraint(15)
                )
                .addTemporalMarker(2.7, () -> {
                    piranhadog.setTeethPwr(0.45);
                })
                .addTemporalMarker(3.2, () -> {
                    piranhadog.setTeethPwr(0);
                })
                .splineToConstantHeading(new Vector2d(14.75, -62), Math.toRadians(75)) // Start off strafing at 75˚
                .splineToConstantHeading(new Vector2d(30, -36), Math.toRadians(0)) // Moving horizontal while dropping pixel
                .splineToConstantHeading(new Vector2d(50, -42), Math.toRadians(0)) // Line up with board
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
//            drive.followTrajectory(buildCorrectionTrajectory(leftTraj.end(), 5, 5));
        }
        else if (nPropPos == goggles2.PROP_MID) {
            drive.followTrajectorySequence(midTraj);
//            drive.followTrajectory(buildCorrectionTrajectory(midTraj.end(), 5, 5));
        }
        else {
            drive.followTrajectorySequence(rightTraj);
//            drive.followTrajectory(buildCorrectionTrajectory(rightTraj.end(), 5, 5));
        }
//        freezeray.autonShootPixel2(this,freezeray.RAY_POS_UNHOLSTER,0.472,0.528,0.59,2000,7000);
        freezeray.autonShootPixel3(this,0.472,0.524,3000,10000);
//        TrajectorySequence moveToOtherside = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .splineToConstantHeading(new Pose2d())
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
