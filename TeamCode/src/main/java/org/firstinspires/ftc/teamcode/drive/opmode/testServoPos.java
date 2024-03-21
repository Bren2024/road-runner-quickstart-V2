package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class testServoPos extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleSwerveDrive drive = new SampleSwerveDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (!isStopRequested()) {
            double pos = Math.atan2(gamepad1.right_stick_x, gamepad1.right_stick_y);
            drive.setMotorPowers(0.2, 0, 0, 0);
            drive.setModuleOrientations(pos, pos, pos, pos);
            telemetry.addData("pos", pos);
            telemetry.addData("current", drive.leftRearModule.getModuleRotation());
            telemetry.addData("target", drive.leftRearModule.getTargetRotation());
            telemetry.addData("allErrors", new double[]{
                    drive.leftFrontModule.getModuleRotation() - drive.leftFrontModule.getTargetRotation(),
                    drive.leftRearModule.getModuleRotation() - drive.leftRearModule.getTargetRotation(),
                    drive.rightFrontModule.getModuleRotation() - drive.rightFrontModule.getTargetRotation(),
                    drive.rightRearModule.getModuleRotation() - drive.rightRearModule.getTargetRotation()
            });

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