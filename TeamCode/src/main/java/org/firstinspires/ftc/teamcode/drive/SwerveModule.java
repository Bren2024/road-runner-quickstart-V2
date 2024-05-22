package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

@Config
public class SwerveModule {
    public static PIDCoefficients MODULE_PID = new PIDCoefficients(0.6, 0, 0.03);

    public static double K_STATIC = 0.2, K_MOTOR = 0;

    public static double MAX_SERVO = 1, MAX_MOTOR = 1;

    //EXPERIMENTAL FEATURES
    public static boolean WAIT_FOR_TARGET = true;
    public static boolean[] waitingForTarget = new boolean[4]; // Shared between all 4 modules
    public static double[] errors = new double[4]; // Shared between all 4 modules

    public static double ALLOWED_COS_ERROR = Math.toRadians(2);

    public static double ALLOWED_BB_ERROR = Math.toRadians(5);

    public static boolean MOTOR_FLIPPING = true;

    public static double FLIP_BIAS = Math.toRadians(15);
    public static double SERVO_TOLERANCE = Math.toRadians(30);


    private DcMotorEx motor;
    private Servo servo;
    private AnalogInput encoder;

    public boolean wheelFlipped = false;

    private double targetRotation = 0;

    private final double DEG_TO_POS = 0.003d; //0=0deg 1=355deg 1/355 = .00282
    private int id;

    public SwerveModule(DcMotorEx m, Servo s, AnalogInput e) {
        motor = m;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);

        servo = s;
        servo.setPosition(.5);

        encoder = e;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public SwerveModule(HardwareMap hardwareMap, String mName, String sName, String eName, int idNum) {
        this(hardwareMap.get(DcMotorEx.class, mName),
                hardwareMap.get(Servo.class, sName),
                hardwareMap.get(AnalogInput.class, eName));
        id = idNum;
    }

    public void setDirection(DcMotorSimple.Direction direction) {  motor.setDirection(direction);  }

    /**
     *
     * @return Module's target rotation in radians
     */
    public double getTargetRotation() {
        return targetRotation;
    }

    /**
     * Approximates module rotation in radians based on encoder voltage
     * @return Module's current rotation in radians
     */
    public double getModuleRotation() {
        return -Math.toRadians((encoder.getVoltage() * 121.8) - 202); // Equation for Voltage to Degrees interpolated from AirShip code
    }

    public double getEncoderVoltage() {
        return encoder.getVoltage();
    }

    public double getWheelPosition() {
        return DriveConstants.encoderTicksToInches(motor.getCurrentPosition());
    }
    public int flipModifier(){
        return MOTOR_FLIPPING && wheelFlipped ? 1 : -1;
    }

    public double getWheelVelocity() {
        return DriveConstants.encoderTicksToInches(motor.getVelocity());
    }

    public boolean getWheelFlipped() {
        return wheelFlipped;
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(runMode, coefficients);
    }
    double lastMotorPower = 0;
    public void setMotorPower(double power) {
        //set current module error in shared array
        double error = getTargetRotation()-getModuleRotation();
        errors[id] = error;

        if(WAIT_FOR_TARGET && !isWithinAllowedError()) {
            // Find minimum power multiplier among all modules
            double minPowerMult = 1;
            for (double e : errors) {
                double powerMult = 1 - Math.abs(Math.sin(Range.clip(e, -Math.PI, Math.PI)));
                if (powerMult < minPowerMult) {
                    minPowerMult = powerMult;
                }
            }

            power *= minPowerMult;
            if (Math.abs(error) > Math.PI/2) {
                power *= -1;
            }
        }
//        if(waitingForTarget[0] || waitingForTarget[1] || waitingForTarget[2] || waitingForTarget[3]) {
//            power = 0;
//        }
        lastMotorPower = power;
        //flip check
        if(MOTOR_FLIPPING) power*=flipModifier();

        motor.setPower(power);
    }

//    public boolean isWithinAllowedError(){
//        double error = encoder.getVoltage();
//        return error < ALLOWED_COS_ERROR || error > 2*Math.PI - ALLOWED_COS_ERROR;
//    }

    public void setServoPosition(double position) {
        servo.setPosition(position);
    }

    public static double MIN_MOTOR_TO_TURN = 0.025;
    public void setTargetRotation(double target) {
        if (Math.abs(lastMotorPower) < MIN_MOTOR_TO_TURN) {
            //add stuff like X-ing preAlign
            return;
        }
        if (MOTOR_FLIPPING) {
            double current = getModuleRotation();

            //normalize for wraparound
            if (current - target > Math.PI) target += (2 * Math.PI);
            else if (target - current > Math.PI) target -= (2 * Math.PI);

            //flip target
            wheelFlipped = (Math.abs(current - target) > (Math.PI / 2 - flipModifier() * FLIP_BIAS))
                    || (Math.abs(target) > 3); // 3: Maximum number of radians the module can turn

            // Unflip target if flipped target is over 3 rad
            if (wheelFlipped && Math.abs(target) < Math.PI - 3) {
                wheelFlipped = false;
            }

            if (wheelFlipped) {
                target = Angle.norm(target + Math.PI);
                if (target > Math.PI) {
                    target -= 2 * Math.PI;
                }
            }
        }
//        // To reset servo position once not moving
//        if (lastMotorPower < MIN_MOTOR_TO_TURN) {
//            target = 0;
//        }
        double targetDeg = Math.toDegrees(target);
        servo.setPosition(degToPos(targetDeg));
        targetRotation = target;
    }

    /**
     * Check if module has finished rotating, and set variable accordingly.
     */
    public boolean isWithinAllowedError() {
        double target = getTargetRotation();

        //get voltage of servos
        double servoPosition = getModuleRotation();
        if (target-SERVO_TOLERANCE<=servoPosition && servoPosition<=target+SERVO_TOLERANCE) {
            //if got here, all rotations complete
            waitingForTarget[id] = false;
        }
        else {
            waitingForTarget[id] = true;
        }
        if (waitingForTarget[0] || waitingForTarget[1] || waitingForTarget[2] || waitingForTarget[3]){
            return false;
        }
        return true;
    }


    public SwerveModuleState asState(){
        return new SwerveModuleState(this);
    }

    public static class SwerveModuleState {
        public SwerveModule module;
        public double wheelPos, podRot;
        public SwerveModuleState(SwerveModule s){
            module = s;
            wheelPos = 0;
            podRot = 0;
        }

        public SwerveModuleState update(){
            return setState(-module.getWheelPosition(), module.getModuleRotation());
        }
        public SwerveModuleState setState(double wheel, double pod){
            wheelPos = wheel;
            podRot = pod;
            return this;
        }
        //TODO add averaging for podrots based off of past values
        public Vector2d calculateDelta() {
            double oldWheel = wheelPos;
            update();
            return Vector2d.polar(wheelPos-oldWheel, podRot);
        }
    }

    private double degToPos(double deg) {
        return 0.5 + (deg * DEG_TO_POS);
    }
    private double posToDeg(double pos) {
        return (pos - 0.5) / DEG_TO_POS;
    }
}