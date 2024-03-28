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
            drive.setMotorPowers(0, 0, 0, 0);
            drive.setModuleOrientations(pos, pos, pos, pos);
            telemetry.addData("pos", pos);
            telemetry.addData("currentL", drive.leftRearModule.getModuleRotation());
            telemetry.addData("targetL", drive.leftRearModule.getTargetRotation());
            telemetry.addData("rawTargetL", drive.leftRearModule.rawTarget);
            telemetry.addData("errorL", drive.leftRearModule.getModuleRotation() - drive.leftRearModule.getTargetRotation());
            telemetry.addData("flippedL", drive.leftRearModule.wheelFlipped ? 1 : 0);
            telemetry.addData("currentR", drive.rightRearModule.getModuleRotation());
            telemetry.addData("targetR", drive.rightRearModule.getTargetRotation());
            telemetry.addData("rawTargetR", drive.rightRearModule.rawTarget);
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