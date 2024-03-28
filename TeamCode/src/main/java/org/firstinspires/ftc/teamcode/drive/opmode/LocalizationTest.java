package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleSwerveDrive drive = new SampleSwerveDrive(hardwareMap);

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        ElapsedTime t = new ElapsedTime();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            new Vector2d(
                            -gamepad1.left_stick_y/2d,
                            -gamepad1.left_stick_x/2d
                                    ).rotated(-drive.getExternalHeading()),
                             -gamepad1.right_stick_x/2d
                    )
            );
            if(gamepad1.right_stick_button) drive.setExternalHeading(0);

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("looptime",1/t.seconds());
            t.reset();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("currentL", drive.leftRearModule.getModuleRotation());
            telemetry.addData("targetL", "raw: %.3f, normalized: %.3f, end: %.3f",
                    drive.leftRearModule.rawTarget, drive.leftRearModule.normTarget, drive.leftRearModule.getTargetRotation());
            telemetry.addData("errorL", drive.leftRearModule.getModuleRotation() - drive.leftRearModule.getTargetRotation());
            telemetry.addData("flippedL", drive.leftRearModule.wheelFlipped ? 1 : 0);
            telemetry.addData("currentR", drive.rightRearModule.getModuleRotation());
            telemetry.addData("targetL", "raw: %.3f, normalized: %.3f, end: %.3f",
                    drive.rightRearModule.rawTarget, drive.rightRearModule.normTarget, drive.rightRearModule.getTargetRotation());
            telemetry.addData("errorR", drive.rightRearModule.getModuleRotation() - drive.rightRearModule.getTargetRotation());
            telemetry.addData("flippedR", drive.rightRearModule.wheelFlipped ? 1 : 0);
//            telemetry.addData("imuuuu", drive.getRawExternalHeading());
//            TelemetryPacket packet = new TelemetryPacket();
//            Canvas fieldOverlay = packet.fieldOverlay();
//
//            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
