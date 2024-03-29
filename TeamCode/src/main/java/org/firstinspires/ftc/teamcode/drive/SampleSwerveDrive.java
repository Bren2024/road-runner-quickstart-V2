package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import static com.kauailabs.navx.ftc.AHRS.getInstance;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.SwerveDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.*;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleSwerveDrive extends SwerveDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(3, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(2, 0, 0);

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private final TrajectorySequenceRunner trajectorySequenceRunner;

    private final TrajectoryVelocityConstraint velocityConstraint;
    private final TrajectoryAccelerationConstraint accelConstraint;

    private final TrajectoryFollower follower;

    public SwerveModule leftFrontModule, leftRearModule, rightRearModule, rightFrontModule;
    public List<SwerveModule> modules;

    private AHRS navx;

    private double gyroVel;

    private final VoltageSensor batteryVoltageSensor;

    public SampleSwerveDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH);

        velocityConstraint = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
        accelConstraint = getAccelerationConstraint(MAX_ACCEL);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftFrontModule = new SwerveModule(hardwareMap, "mtrLeftFront", "srvoLeftFront", "anaRotationLF", 0);
        leftRearModule = new SwerveModule(hardwareMap, "mtrLeftBack", "srvoLeftBack", "anaRotationLB", 1);
        rightFrontModule = new SwerveModule(hardwareMap, "mtrRightFront", "srvoRightFront", "anaRotationRF", 2);
        rightRearModule = new SwerveModule(hardwareMap, "mtrRightBack", "srvoRightBack", "anaRotationRB", 3);

        modules = Arrays.asList(leftFrontModule, leftRearModule, rightRearModule, rightFrontModule);


        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO instantiate hardware
        navx = getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData, (byte) 50);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        rightFrontModule.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearModule.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
//         setLocalizer(new BetterSwerveLocalizer(this::getExternalHeading, leftFrontModule, leftRearModule, rightRearModule, rightFrontModule));
        setLocalizer(new TwoWheelTrackingLocalizer(hardwareMap, this));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID, batteryVoltageSensor);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, velocityConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, velocityConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, velocityConstraint, accelConstraint);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH), getAccelerationConstraint(MAX_ACCEL),
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public static TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose, double startHeading) {
        return new TrajectorySequenceBuilder(
                startPose,
                startHeading,
                getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH), getAccelerationConstraint(MAX_ACCEL),
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    private void updateModules() {
        leftFrontModule.update();
        leftRearModule.update();
        rightFrontModule.update();
        rightRearModule.update();
    }

    public void update() {
        updateModules();
        updateGyroVelocity();
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();

    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (SwerveModule module : modules) {
            module.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (SwerveModule module : modules) {
            module.setZeroPowerBehavior(zeroPowerBehavior);
        }

    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (SwerveModule module : modules) {
            module.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (SwerveModule module : modules) {
            wheelPositions.add(module.getWheelPosition());
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (SwerveModule module : modules) {
            wheelVelocities.add(module.getWheelVelocity());
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFrontModule.setMotorPower(v);
        leftRearModule.setMotorPower(v1);
        rightFrontModule.setMotorPower(v2);
        rightRearModule.setMotorPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
//        return 0;
        return -Math.toRadians(lastHeading);
    }

    double lastHeading = 0;
    long lastTimestamp = System.currentTimeMillis();
    public void updateGyroVelocity() {
        double heading = navx.getYaw();
        long timestamp = System.currentTimeMillis();
        double headingDifference = heading-lastHeading;
        if (headingDifference > 180) {
            headingDifference -= 360;
        }
        gyroVel = headingDifference / (timestamp-lastTimestamp);
        lastHeading = heading;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // To work around an SDK bug, use -zRotationRate in place of xRotationRate
        // and -xRotationRate in place of zRotationRate (yRotationRate behaves as
        // expected). This bug does NOT affect orientation.
        //
        // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
        return (double) gyroVel;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    @NonNull
    @Override
    public List<Double> getModuleOrientations() {
        List<Double> moduleOrientations = new ArrayList<>();
        for (SwerveModule module : modules) {
            moduleOrientations.add(module.getModuleRotation());
        }
        return moduleOrientations;
    }


    @Override
    public void setModuleOrientations(double p, double p1, double p2, double p3) {
        leftFrontModule.setTargetRotation(p);
        leftRearModule.setTargetRotation(p1);
        rightRearModule.setTargetRotation(p2);
        rightFrontModule.setTargetRotation(p3);
    }
}