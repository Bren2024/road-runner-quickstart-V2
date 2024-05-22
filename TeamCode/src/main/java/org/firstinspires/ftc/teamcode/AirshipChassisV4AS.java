package org.firstinspires.ftc.teamcode;
//From the Kauai Labs Docs
// https://pdocs.kauailabs.com/navx-micro/software/libraries/android-library-2016-version/
//Add the following to  teamcode/build.release.gradle
//  Add the following to the repositories section
//   repositories {
//       flatDir {
//           dirs 'libs', 'C:\\Users\\Robot\\navx-micro\\android\\libs'
//       }
//   }
// Add the following to to dependencies section
//   dependencies {
//         //compile (name:'navx_ftc-release', ext:'aar')
//         implementation (name:'navx_ftc-release', ext:'aar')
//   }
//
//after entering the above, the following lines should compile
//Revision 2021-09  teamcode may only have build.gradle.. so put above in build.gradle



import static com.kauailabs.navx.ftc.AHRS.getInstance;
//the next line is from the FIRST library for the navX.  This code uses both

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



/**
* A field-centric omni drive AND a robot-centric tank drive robot chassis
* Mecanum wheels or swerve modules, 4 motors w/encoders, and a Kauai Labs navX required
* @version 9
* @author  FTC Teams 6109 and 5159
*
*/
public class AirshipChassisV4AS {


    private AHRS navX;  //Kauai Labs library
    private navXPIDController yawPIDController;
    navXPIDController.PIDResult yawPIDResult;

    //Kauai Labs navX constants
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private final double TARGET_ANGLE_DEGREES = 0.0d;
    //private final double TOLERANCE_DEGREES = 2.0d; //when within this tolerance, no more heading correction
    //corrections are made
    private final int NAVX_TIMEOUT = 200;

    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0d;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0d;

  
    //FIRST library
    IntegratingGyroscope gyro;

    //select library
    private boolean NAVX_KL_LIB=false;

    private SwerveV4AS swerve;
    private MecanumV4AS mec;


    //constants for chassis

    public final int CHASSIS_MEC = 1;
    public final int CHASSIS_SWERVE = 2; 

    public final int CHASSIS_LEFT_FWD = 1; //Left wheels move forward when apply positive voltage
    //Includes NeverRest 20 orbitals
    public final int CHASSIS_RIGHT_FWD = 2; //Right wheels move forward when apply positive voltage
    //Includes NeverRest 3.7 orbitals
    public final boolean USE_KL_LIB = true;
    public final boolean USE_QC_LIB = false;



    //Drive Constants
    private final float STICK_DEADZONE = (float) .33;
    private final boolean STICK_RIGHT = false, STICK_LEFT = true;
    private final boolean DIR_STICK = false, ROT_STICK = true;
    public final int CCW = -1, CW = 1, NOSPIN = 0;
    public final int STICK_REQ = 1, BUTTON_REQ=2, NO_REQ_PREV_STICK=3, NO_REQ_PREV_BTN=4, REQ_DONE=0, REQ_CHECK = -1;
    public final int CRAWL_FWD=0, CRAWL_BACK=2, CRAWL_LEFT=3, CRAWL_RIGHT=4, CRAWL_NONE=-1;
    public final int PREDEF_ROTATE_NONE=999;

    public final int OPT_NONE=0, OPT_AUTOCORRECT_DIR_OFF=1;

    //Gamepad Constants
    public final long CONTROL_SENSITIVITY=100;

    //NavX constants
    private final int NAVX_NONE = 0, NAVX_KAUAILABS_LIB = 1, NAVX_QC_LIB = 2, NAVX_ERROR = 9;

    //Autonomous Constants
    public final int AUTON_FWD = 0;
    public final int AUTON_REV = 1;
    public final int AUTON_LEFT = 2;
    public final int AUTON_RIGHT = 3;

    final int AUTON_TIMEOUT = -1;

    //this class name
    private String gstrClassName=this.getClass().getSimpleName();
    private boolean gbDebug=false;

    //NavX globals
    private int gnNavXStatus = NAVX_NONE;
    private double gdYawAngleToHold=0;
    


    //below only for DIY`YawCorrection

    private double gdYAW_P =0;

    //gamepad global variables
    private boolean gbDirectionStick=STICK_LEFT,gbRotationStick=STICK_RIGHT;
    private int gnCrawling=CRAWL_NONE;
    private int gnPreDefRotate=0;
    private long glLastControlModePress=0;
    
    //robot global variables
    private int gnChassis;
    private double gdDirectionStickPolar=0d; //stores the polar angle of the direction stick (which could be right or left)
    private double gdRobotMoveFromDirCntlPolar=0d;//field oriented direction to move robot
    private double gdDirMagnitude, gdRotMagnitude; //robot directional and rotational magnitue
    private double gdToleranceDeg;
    private double gdSlowPwr=.5, gdFastPwr=1.0; //initially at .6
    private double gdSlowMomentumAdjDeg=4d,gdFastMomentumAdjDeg=8d; 
    private double gdDirPwr = gdFastPwr;
    private double gdRotPwr = 0;
    private double gdMomentumAdjustmentDeg=gdFastMomentumAdjDeg;
    private int gnRobotSpin=NOSPIN; //CCW, CW, NOSPIN the current spin of robot
    private int gnReqSpin=REQ_DONE; //the current request by user, used to determine a new rotation request
    private double gdRotPwrScale;
    private double gdMinYawPwr;
    private boolean gbAutoCorrectDir=true;
    private double gdNavXOutput=0;

    private int gnBrakeDist;

    public AirshipChassisV4AS() {

    }
    /**
     * Initialize the chassis. Must be called after class creation
     * "Chassis" consists of the motors plus the navX
     *
     * @param opMode               opMode calling this method
     * @param nReqLayout           layout for chassis.  Either CHASSIS_LEFT_FWD  or CHASSIS_RIGHT_FWD
     *                             CHASSIS_LEFT_FWD: left wheels move forward with pos power applied
     *                             and Right wheels move back with pos power applied
     *                             CHASSIS_RIGHT_FWD: Right wheels move forward with pos power applied
     *                             and left wheels move back with pos power applied
     *                             Note: incorrect requested layout defaults to CHASSIS_LEFT_FWD
     * @param dDirFastPwr          Fast Power set by bumper press.  Used to set directional power, scaled for rotation power
     * @param dDirSlowPwr          Slow Power set by trigger press. Same as above
     * @param dRotPwrPct           Percent of above to apply to rotoational power
     */
    /**
     * Initialize the chassis. Must be called after class creation
     * "Chassis" consists of the motors plus the navX
     *
     * @param opMode               opMode calling this method
     * @param nChassis             Kind of chasis, either Mec or Swerve
     * @param nReqLayout           layout for chassis.  Either CHASSIS_LEFT_FWD  or CHASSIS_RIGHT_FWD
     *                             CHASSIS_LEFT_FWD: left wheels move forward with pos power applied
     *                             and Right wheels move back with pos power applied
     *                             CHASSIS_RIGHT_FWD: Right wheels move forward with pos power applied
     *                             and left wheels move back with pos power applied
     *                             Note: incorrect requested layout defaults to CHASSIS_LEFT_FWD
     * @param strNavXName          name of navX in robot config
     * @param bUseKLLib            use KL library if true, use QC library if false
     * @param strMtrLFName         name of Left Front motor in robot config
     * @param strMtrLBName         name of left back motor in robot config
     * @param strMtrRFName         name of right front motor in robot config
     * @param strMtrRBName         name of right back motor in robot config
     * @param odoHoriz             name of horizontal odometry pod
     * @param odoVert               name of vertical odometry pod
     * @param strSrvoLFName         name of Left Front servo in robot config
     * @param strSrvoLBName         name of left back servo in robot config
     * @param strSrvoRFName         name of right front servo in robot config
     * @param strSrvoRBName         name of right back servo in robot config
     //* @param strDSRight            the name of the right 2m dist sensor or none
     //* @param strDSLeft            the name of the right 2m dist sensor or none
     //* @param strDSCenter          the name of the Center REV 2m dist sensor or none
     * @param nOptions            bit evaluated option string;
     * @param dFastMomentumAdjDeg increase if robot tries to adjust too much when release dir
     * @param dSlowMomentumAdjDeg increase if robot tries to adjust too/ much when release dir
     * @param dToleranceDeg       Set tolerance for corrections
     * @param dFastPwr            Fast Power set by bumper press
     * @param dSlowPwr             Slow Power set by trigger press
     * @param dRotScalePwr       Multiplier to get Rotational pwr     
     * @param dMinRotPwr          Minimum Power to spin robot
     * @param dRotP               P multiplier PID yaw rotation control
     * @param nSlowDownDist       Distance (encoder count) to start slowing down
     * @param dMinDirPwr         minimum pwr to apply when moving in linear direction
     * @param dDirP               P multiplier PID linear Directional control
     * @param nBrakeDist          distance remain to apply break power
     * @param dBrakePwr           Pwr (taken negatively) to use as breaking power
     * @param lBrakeTime          Amount of MS to break;
     * @param nLinearAdjDist      apply correction when  linear dist exceeded
     * @param lRotationPause      pause between rotating wheel and moving
     * <p>
     */
    public void initialize(OpMode opMode, int nChassis, int nReqLayout, String strNavXName,
        boolean bUseKLLib,
        String strMtrLFName, String strMtrLBName,String strMtrRFName, String strMtrRBName,
        DcMotor odoHoriz, DcMotor odoVert,
        String strSrvoLFName, String strSrvoLBName,String strSrvoRFName, String strSrvoRBName,
        //String strDSLeft, String strDSRight, String strDSCenter,
        int nOptions,
        double dFastMomentumAdjDeg,double dSlowMomentumAdjDeg, double dToleranceDeg,
        double dFastPwr, double dSlowPwr, double dRotPwrScale, double dMinRotPwr, double dRotP,
        int nSlowDownDist, double dMinDirPwr, double dDirP,int nBrakeDist,double dBrakePwr,long lBrakeTime,
        int nLinearAdjDist, long lRotationPause) {

        NAVX_KL_LIB = bUseKLLib;

        initNavX(opMode,strNavXName);

        gbDirectionStick=STICK_LEFT;
        if(nChassis!=CHASSIS_SWERVE) {
            opMode.telemetry.addData(gstrClassName,"invalid Chassis");
            opMode.telemetry.update();
            return;
        } else {
            gnChassis=nChassis;
            swerve = new SwerveV4AS(opMode,nReqLayout,strMtrLFName, strMtrLBName,
                strMtrRFName, strMtrRBName,
                odoHoriz,odoVert,
                strSrvoLFName, strSrvoLBName, strSrvoRFName, strSrvoRBName,
                lRotationPause);
            swerve.setDebug(false);

        }

        gdFastMomentumAdjDeg=dFastMomentumAdjDeg;
        gdSlowMomentumAdjDeg=dSlowMomentumAdjDeg;
        gdToleranceDeg=dToleranceDeg;
        gdSlowPwr=dSlowPwr;
        gdFastPwr=dFastPwr;
        gdRotPwrScale=dRotPwrScale;
        gdMinYawPwr=dMinRotPwr;
        gdYAW_P =dRotP;

        gdDirPwr=gdFastPwr;
        gdRotPwr=gdDirPwr*gdRotPwrScale;

        gnBrakeDist = nBrakeDist;

        if((nOptions&OPT_AUTOCORRECT_DIR_OFF)==OPT_AUTOCORRECT_DIR_OFF) {
            //turn off auto correction
            gbAutoCorrectDir=false;
        } else {
            //turn on auto correction
            gbAutoCorrectDir=true;
        }


    }
     /**
     * main method call for teleop OpModes
     * Checks if user selected omni or tank control mode for chassis operation
     * omni control: field oriented movement, robot can rotate while moving
     * tank control: robots moves fwd, back, left and right OR rotates
     * Checks throttle settings. omni and tank modes use throttle to set robot speed
     * Checks if user changed omni or tank mode control options
     * omni options: left hand driver or right hand driver
     * tank options: forward mode, or reverse mode (switches front and back of robot)
     * Moves robot using tank mode or omni mode, based on above settings above
     * <p>
     * Control mapping on gamepad
     * <p>
     * Triggers and Bumpers change the robot's throttle settings
     * ==LT==(set throttle slower)      ==RT==(set throttle to slow)
     * ==LB==(set throttle faster)       ==RB==(Set throttle to fast)
     *       ----------------               --------------
     *     /                 \------------/                \
     *    /                                                  \
     *  /            up       Back     Start        Y          \   
     * /        left   right    Logitec          X     B        \  
     * |            down                            A           |  
     * |                                                        |
     * |              ^                            ^            |
     * |              |                            |            |
     * |      <-- Lt Stick -->             <-- Rt Stick -->     |  <---The gamepad sticks are also buttons. this sets puts the contols in right or left hand mode 
     * |              |                            |            | 
     * |              v                            v            | 
     * |                 /---------------------\                | 
     * |               /                         \              |
     * \              /                           \             /  
     * \            /                             \           /    
     * \ ........./                               \---------/     
     * <p>
     * Buttons near the top of the gamepad control "crawl"
     * Crawl is for robot oriented fine tuning position movement, such as moving slightly left to aim
     * omni mode
     * left hand option- up,down.left,right is robot direction (robot oriented); X,B spins CW,CCW
     * right hand option- Y,A,X,B is direction (robot oriented), left,right spins CW,CCW
     * tank mode
     * forward option- Y,A,X,A moves robot fwd,back,left,right; left,right spins CW,CCW
     * reverse option- A,X,B,X moves robot fwd,back,left,right; right,left spins CW,CCW
     *
     * @param opMode opMode calling this method
     * @param gamepad driver's gamepad, typically gamepad1
     *                return none
     */
    public void operate(OpMode opMode,Gamepad gamepad) {
if(gbDebug) System.out.printf("Operating %s\n", gstrClassName);
if(gbDebug) System.out.printf("  Intiial LB Dir:%.2f(%.2f) Rot:%.2f(%.2f) Res:%.2f(%.2f)\n",
            swerve.getDirPolar(swerve.LB),swerve.getDirMagnitude(swerve.LB),
            swerve.getRotPolar(swerve.LB), swerve.getRotMagnitude(swerve.LB),
            swerve.getResPolar(swerve.LB), swerve.getResMagnitude(swerve.LB));
        chkControlOptions(gamepad);
        chkThrottle(gamepad);
        setRobotMotion(opMode, gamepad);
if(gbDebug) System.out.printf("  Settings: joystick (%.2f,%.2f) Button %d imu of %.2f\n",
            gamepad.left_stick_x,gamepad.left_stick_y,chkCrawlButtons(gamepad),getNavXYaw()) ;
if(gbDebug) System.out.printf("  Direction Stick:%.2f",gdDirectionStickPolar) ;
if(gbDebug) System.out.println("  Robot Yaw:"+getNavXYaw()) ;

        //at this point:
        // 1.  Have the power and linear direction for robot to travel 
        // 2.  Have the power and rotation direction to rotate the robot
        if(gnChassis==CHASSIS_SWERVE) {
            if(gbDebug) System.out.printf("  After LB Dir:%.2f(%.2f) Rot:%.2f(%.2f) Res:%.2f(%.2f)\n",
                swerve.getDirPolar(swerve.LB),swerve.getDirMagnitude(swerve.LB),
                swerve.getRotPolar(swerve.LB),swerve.getRotMagnitude(swerve.LB),
                swerve.getResPolar(swerve.LB),swerve.getResMagnitude(swerve.LB));
            swerve.operate(opMode, gdRobotMoveFromDirCntlPolar, gdDirMagnitude, gnRobotSpin, gdRotMagnitude);
        } else {
            mec.operate();
        }
    }

    /**
     * Set the robot motion
     * robot motion has 2 components
     * direction - robot moves in the field oriented direction of the direction stick or crawl btns
     * rotation - robot rotates clock wise (CW) or counter clock wise (CCW)
     * the robot moves in a field oriented direction and can rotate at the same time 
     *
     * This method therefore sets two vectors
     *    Direction vector
     *        gdDirMagnitude (zero when no control pressed)
     *        gdRobotMoveFromDirCntlPolar (desired angle in polar)
     *    Rotation vector
     *        gdRotMagnitude (set when control pressed, also used for auto correct
     *        gnRobotSpin(CW,CCW, 0... sets the wheels at 45deg,135deg or 0deg polar)
     *
     * Note that this only sets vectors, it does not power the wheels
     * @param opMode          opmode to use
     * @param gamepad         driver's gamepad, typically gamepad1
     * 
     */
    private void setRobotMotion(OpMode opMode,Gamepad gamepad) {

        //find if user requested linear motion by moving direction stick or pressing crawl buttons
        setRobotDirection(opMode,gamepad);
        //opMode.telemetry.update();

        //set rotation from user request or from correction
        setRobotRotation(opMode,gamepad);

    }

    /**
     * Set the robot rotation based on the rotation stick position or rotation buttons
     * or by correcting the robot orientation 
     *        sets gnRobotSpin (CCW, CW, NOSPIN)
     *        sets gnReqSpin (STICK_REQ,BUTTON_REQ,NO_REQ_PREV_STICK,NO_REQ_PREV_BTN,REQ_DONE)
     *        sets gdRotMagnitude;
     *
     * @param gamepad        driver's gamepad, typically gamepad1
     */
    private void setRobotRotation(OpMode opMode,Gamepad gamepad) {
        //Yaw stick rotates the robot more clockwise or counter clockwise from its current position
        //1.  When the yaw stick is moved (not in dead zone) rotate robot
        //2.  When the stick is in the dead zone (not moved), the user wants the robot to stop
        //    turning.  Use the NavX Yaw PID controller (or equivalent)
        //    to hold the position set when the yaw stick is first released
        //    Note:  The robot will continue to rotate beyond the point that the user releases the
        //    stick because of momentum.  Therefore, set the yaw to hold slightly beyond the point
        //    that the user releases the stick
        //3.  The user can also press one of the pre-defined rotation buttons, rotation power is set
        //    by correctional power

        double dStickX;
        double dDistToRotate=0,dCurrentPolar=0;
   
        int nReqSpin=REQ_DONE, nRobotSpin=NOSPIN;

if(gbDebug) System.out.printf("set robot rotation spin before:%d",gnRobotSpin);
        //set Yaw according to stick and buttons
        //check if yaw stick is released
        if (!chkStickDeadZone(gamepad,ROT_STICK)) {
            //stick was moved because we are not in dead zone
            ///CAUTION!!! Joysticks front/back are REVERSED!!!! left/right is okay-
            //     -          -
            //  -  L  +    -  R
            //     +          +
            if (gbRotationStick == STICK_LEFT)
                dStickX = (double) gamepad.left_stick_x;
            else
                dStickX = (double) gamepad.right_stick_x;

            //when dStickX is negative, need to rotate counter clock wise
            if (dStickX < 0) {
                nReqSpin=STICK_REQ;
                nRobotSpin=CCW;
            } else if (dStickX > 0) {
                nReqSpin=STICK_REQ;
                nRobotSpin=CW;
            }
if(gbDebug) System.out.printf("...rotation stick moved.... ");
        }  else { //stick is in dead zone
            //stick not  moved so check buttons
            dCurrentPolar=getNavXYaw();
            //if got here, no stick press
            //Now check buttons for rotation
            //check if dpad or buttons were pressed  (overrides stick)
            //
            //       Up                      Y
            //   Left  Right             X       B
            //      Down                     A
            //
            //     L-Stick               R-Stick
            if (gbRotationStick == STICK_LEFT) {
                //if rotation stick is left, rotation buttons on dpad above left stick
if(gbDebug) System.out.printf("...checking left side rotation buttons ... ");
                if (gamepad.dpad_left) { //-45 degrees (robot oriented) request;
                //need to turn the robot to -45 degrees (robot oriented)
if(gbDebug) System.out.printf("...rotation button left pressed.... ");
                    nReqSpin=BUTTON_REQ; 
                    dDistToRotate=calcDistToRotateDegrees(-45d,dCurrentPolar);
                    //enableNavXYawPID(true);
                    setpointNavXYaw(-45d);
                } else if (gamepad.dpad_right) { //45 degrees non-field oriented request
if(gbDebug) System.out.printf("...rotation button right pressed.... ");
                    nReqSpin=BUTTON_REQ; 
                    dDistToRotate=calcDistToRotateDegrees(45d,dCurrentPolar);
                    //enableNavXYawPID(true);
                    setpointNavXYaw(45d);
                } else if (gamepad.dpad_up) { //0 degrees non-field oriented request
if(gbDebug) System.out.printf("...rotation button up pressed.... ");
                    nReqSpin=BUTTON_REQ; 
                    dDistToRotate=calcDistToRotateDegrees(0d,dCurrentPolar);
                    //enableNavXYawPID(true);
                    setpointNavXYaw(0d);
                } else if (gamepad.dpad_down) { //180 degrees non-field oriented request
if(gbDebug) System.out.printf("...rotation button down pressed.... ");
                    nReqSpin=BUTTON_REQ; 
                    dDistToRotate=calcDistToRotateDegrees(180d,dCurrentPolar);
                    //enableNavXYawPID(true);
                    setpointNavXYaw(180d);

                }
            } else  { //bRotationStick == STICK_RIGHT
                //if yaw stick is right, yaw buttons are above right stick
if(gbDebug) System.out.printf("...checking righ side rotation buttons ... ");
                if (gamepad.x) { //left button -45 non-field oriented request
if(gbDebug) System.out.printf("...rotation button left pressed.... ");
                    nReqSpin=BUTTON_REQ; 
                    dDistToRotate=calcDistToRotateDegrees(-45d,dCurrentPolar);
                    //enableNavXYawPID(true);
                    setpointNavXYaw(-45d);
                } else if (gamepad.b) { //right button 45 non-field oriented request
if(gbDebug) System.out.printf("...rotation button right pressed.... ");
                    nReqSpin=BUTTON_REQ; 
                    dDistToRotate=calcDistToRotateDegrees(45d,dCurrentPolar);
                    //enableNavXYawPID(true);
                    setpointNavXYaw(45d);
                } else if (gamepad.y) { //up button 0 deg non-field oriented request
if(gbDebug) System.out.printf("...rotation button up pressed.... ");
                    nReqSpin=BUTTON_REQ; 
                    dDistToRotate=calcDistToRotateDegrees(0d,dCurrentPolar);
                    //enableNavXYawPID(true);
                    setpointNavXYaw(0d);
                } else if (gamepad.a) { //down button 180 non field oriented request
if(gbDebug) System.out.printf("...rotation button down pressed.... ");
                    nReqSpin=BUTTON_REQ; 
                    dDistToRotate=calcDistToRotateDegrees(180d,dCurrentPolar);
                    //enableNavXYawPID(true);
                    setpointNavXYaw(180d);
                }
            }
            //recall that stick is in deadzone
            //check if button pressed  
            if(nReqSpin==BUTTON_REQ) {  
                //if got here a button was pressed
                if(dDistToRotate>0) nRobotSpin=CW;
                else if(dDistToRotate<0) nRobotSpin=CCW;
                //Log.w(gstrClassName,"Button Req ");

                /*
                long lTimestamp=System.currentTimeMillis();
                while((System.currentTimeMillis()-lTimestamp)<5000) {
                    opMode.telemetry.addData(gstrClassName,"Distance to rotate:%.2f",dDistToRotate) ;
                    opMode.telemetry.addData(gstrClassName,"   spin is %d",nRobotSpin) ;
                    opMode.telemetry.update();
                }
                 */


            }

        } //done looking at sticks and buttons

        //Log.w(gstrClassName,"No Button or stick Req ");
        if(gbDebug) System.out.printf(" after stick and button check:%d",nReqSpin);
        //At this point, nRobotSpin will be CW, CCW, or NO_SPIN (initial state)
        //nReqSpin will show if stick moved (STICK_REQ), button pressed(BUTTON_REQ)
        // or still be in REQ_DONE (initial state)
        //
        //Now need to do several things
        //if stick or button moved, gnRobot spin needs to be set
        //else
        //  if stick was just release, need to set the target for the robot to spin
        //  Recall that a button press also sets the target for the robot to spin
        //
        //nReqSpin will show if stick moved (STICK_REQ), button pressed(BUTTON_REQ)
        // or it initial state (REQ_DONE)
        if(nReqSpin==STICK_REQ) {
if (gbDebug) System.out.printf("now have a different req spin:%d\n", nReqSpin);
            gnRobotSpin =nRobotSpin;  //set robot spin to new spin movement
            gnReqSpin = nReqSpin; //update for the next time, keeps track of new request, not the robot spin
            gdRotMagnitude = gdRotPwr; //get the rotational magnitude from the throttle settings
            return;
        } else if(nReqSpin==BUTTON_REQ) { //predefined movement
            gnRobotSpin = nRobotSpin;  //set robot spin to new spin movement
            gnReqSpin = nReqSpin; //update for the next time, keeps track of new request, not the robot spin
            gdRotMagnitude = getNavXCorrectionPwr(opMode); //get the rotational magnitude from the throttle setting
            /*
            long lTimestamp=System.currentTimeMillis();
            while((System.currentTimeMillis()-lTimestamp)<5000) {
                opMode.telemetry.addData(gstrClassName,"ButtonReq:corrective pwr:%.2f spin:%d dist:%.2f",getNavXCorrectionPwr(opMode),nRobotSpin,dDistToRotate) ;
                opMode.telemetry.update();
            }

             */


            return;
        } else {  //nReqSpin is at its initial state REQ_DONE
            //got here, so stick not moved nor button pressed
            //check if this is the first time stick was released
            if (gnReqSpin == STICK_REQ) {  //if stick previously was moved
                //prev:STICK_REQ  now REQ_DONE
                if (gbDebug) System.out.printf("No spin request now, had stick request before..");
                //now nothing pressed, previously there was a stick move 
                //the user just released  joystick (since nReqSpin is NO_REQ and gnReqSpin is STICK_REQ) 
                //   so need to stop and hold spin
                //note that there will be a a lag to stop so add to the stopping point
                //Set the target yaw when the stick or button is RELEASED after being pressed
                //From the user's stand point, robot rotates until the button/stick is released
                //and thereafter should keep the same yaw
                //NOTE:  The robot's momentum will keep it spinning slightly beyond when the user releases
                //       the stick.  Will try to guess where the robot shold actually stop
                //       INSTEAD of when the user released the stick.
                //       The yaw at which the robot stops spinning is beyond the yaw at which the
                //       user released the stick/btn.  Will therefore try to maintain the yaw at which the
                //       robot stopped spinning.
                //
                //The momentum adjustment is throttle dependent
                //   gdMomentumAdjustmentDeg is therefore set when the user adjusts the throttle
                //   we will use that value here
                if (gbDebug) System.out.printf("setting target...");
                //enableNavXYawPID(true);//turn on
                if (gnRobotSpin == CCW) {
                    setpointNavXYaw(getNavXYaw() - gdMomentumAdjustmentDeg);
                } else if (gnRobotSpin == CW) {
                    setpointNavXYaw(getNavXYaw() + gdMomentumAdjustmentDeg);
                }
                gnReqSpin = NO_REQ_PREV_STICK; //update for the next time, keeps track of new request, not the robot spin
                //leave gnRobotSpin as it was set before
if (gbDebug) System.out.printf("target is %.2f (robotyaw:%.2f, adjustment=%.2f\n", gdYawAngleToHold, getNavXYaw(), gdMomentumAdjustmentDeg);
            } else if (gnReqSpin == BUTTON_REQ) {  //if button was previously was moved
                //prev:BUTTON_REQ  now REQ_DONE
                gnReqSpin = NO_REQ_PREV_BTN; //update for the next time, keeps track of new request, not the robot spin
            }
            //at this point user has not touched stick or button
            //see if its time to turn off the auto movement from stick or button
            if((gnReqSpin==NO_REQ_PREV_BTN) || (gnReqSpin==NO_REQ_PREV_STICK)) {
                gdRotMagnitude = getNavXCorrectionPwr(opMode); //keep going to until get to target
                //corrective power is tricky because the correction power returns pos for CW neg for CCW based on setpoint
                //the swerve sets the CW and CCW and rotation power based on user input

                //long lTimeStamp=System.currentTimeMillis();
                //while((System.currentTimeMillis()-lTimeStamp)<1000) {
                    opMode.telemetry.addData(gstrClassName, "Norep.. prev button or stick and correction is %.2f", gdRotMagnitude);
                    opMode.telemetry.addData(gstrClassName, "   target:%.2f pos:%.2f spin:%d", gdYawAngleToHold,getNavXYaw(),gnRobotSpin);

                opMode.telemetry.update();
                //}

                //see if correctional power is 0
                /*
                if((gdYawAngleToHold-getNavXYaw())<=gdToleranceDeg) {
                    gnReqSpin=REQ_DONE;
                    gnRobotSpin=NOSPIN;
                    gdRotMagnitude=0;
                }

                 */
                if(gdRotMagnitude == 0) {
                    gnReqSpin=REQ_DONE;
                    gnRobotSpin=NOSPIN;
                    gdRotMagnitude=0;

                    /*
                    long lTimestamp=System.currentTimeMillis();
                    while((System.currentTimeMillis()-lTimestamp)<2000) {
                        opMode.telemetry.addData(gstrClassName,"Norep.. prev button or stick and correction is 0") ;
                        opMode.telemetry.update();
                    }
                     */


                }
                return;
            }
            //check if crawling
            if(gnCrawling!=CRAWL_NONE) {
                gnRobotSpin=NOSPIN;
                gdRotMagnitude=0;
                gnReqSpin=REQ_DONE;
                return;
            }

            //at this point user has not touched stick or button
            //autocorrect direction if needed
            if(gbAutoCorrectDir) {
                //had to have previously set the setpoint target
                gdRotMagnitude = getNavXCorrectionPwr(opMode); //keep going to until get to target
                nReqSpin=REQ_DONE;
            }


        }//end of reqstick at its initial state
        // end of checking for button press or stick move

    }

    /**
     * get the navx Correction power from the KL libraires
     * @param opMode
     * @return
     */
    private double getNavXCorrectionPwrKL(OpMode opMode) {
        double output=0;
         
        try {

            //Wait for new Yaw PID output values, then update the motors
            // with the new PID value with each new output value.

            final double TOTAL_RUN_TIME_SECONDS = 30.0;
            int DEVICE_TIMEOUT_MS = 500;
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            long lTimestamp=System.currentTimeMillis();
            while ( ((System.currentTimeMillis()-lTimestamp) < (TOTAL_RUN_TIME_SECONDS*1000)) &&
                    !Thread.currentThread().isInterrupted()) {
                opMode.telemetry.addData(gstrClassName,"NavX yawPID reading");
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        opMode.telemetry.addData(gstrClassName,"on target");
                        output=0d;
                        gdNavXOutput=output;

                        //Log.w(gstrClassName,"NavX is on target");
                        /*
                        long lTimestamp=System.currentTimeMillis();
                        while((System.currentTimeMillis()-lTimestamp)<1000) {
                            opMode.telemetry.addData(gstrClassName,"ON TARGET! ") ;
                            opMode.telemetry.update();
                        }

                         */

                    } else {  //not on target
                        output = yawPIDResult.getOutput();
                        opMode.telemetry.addData(gstrClassName,"not on target output=%.2f",output);
                        gdNavXOutput=output;
                    }
                    opMode.telemetry.addData(gstrClassName,"    Got update ");
                    break;  
                } else {//no new update
                    //no update available
                    //Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                    opMode.telemetry.addData(gstrClassName,"    no update avail, last:%.2f",gdNavXOutput);
                    output=gdNavXOutput;

                }
                opMode.telemetry.addData(gstrClassName,"Yaw:%.2f",navX.getYaw());
                opMode.telemetry.update();
            }
        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
            output=0;
            gdNavXOutput=output;
        }
        finally {
            //navx_device.close();
            opMode.telemetry.addData(gstrClassName,"NavX get corrective power complete, output:%.2f",output);
        }
        opMode.telemetry.update();
        return output;

    }
    /**
     * 
     * Get the power needed for correcting the robot using the NavX
     * @param opMode opmode to use
     * @return power to apply to left wheels, use a neg valur of this power to apply to right wheels
     */
    public double getNavXCorrectionPwr(OpMode opMode) {
        //prior to calling this, the user call setpointNavXYaw to set the target angle
        //   the difference between the robot's current angle and the target angle
        //   determines the magnitude and sign of the output returned.

        long lTimestamp = System.currentTimeMillis();
        double dOutput;
        if (gnNavXStatus == NAVX_KAUAILABS_LIB) {
            dOutput=getNavXCorrectionPwrKL(opMode);
            /*
            if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
                opMode.telemetry.addData(gstrClassName,"KL Update Avail");
                if (yawPIDResult.isOnTarget()) {

                    while((System.currentTimeMillis()-lTimestamp)<5000)  {
                        opMode.telemetry.addData(gstrClassName,"update avail, on target%.2f",0);
                        opMode.telemetry.update();
                    }
                    gdNavXOutput=0;
                    return 0d;
                } else {
                    double output = yawPIDResult.getOutput();
                    gdNavXOutput=output;
                    while((System.currentTimeMillis()-lTimestamp)<5000)  {
                        opMode.telemetry.addData(gstrClassName,"update avail, not on target%.2f",output);
                        opMode.telemetry.addData(gstrClassName,"KL output:%.2f",output);
                        opMode.telemetry.update();
                    }
                    return output;
                }
            } else {
                opMode.telemetry.addData(gstrClassName,"KL No Update Avail");
                opMode.telemetry.addData(gstrClassName,"KL output:%.2f",gdNavXOutput);
                // No sensor update has been received since the last time
                // the loop() function was invoked.  Therefore, there's no
                // need to update the motors at this time.
                opMode.telemetry.addData(gstrClassName,"NavX update NOT Avail");
                return gdNavXOutput;
            }

             */

        } else if (gnNavXStatus == NAVX_QC_LIB) {
            dOutput= calcDIYYawCorrectionPwr(opMode);
        } else {
            //opMode.telemetry.addData(gstrClassName,"ERR");
            return 0;
        }
        if(gnChassis==CHASSIS_SWERVE) {
            //if got here working with swerve chassis
            //output is based on neg pwr CCW, pos pwr for CW
            //swerve does not use this model, so must correct:
            if (gnRobotSpin==CW)  {
                //if got here navx will give a positive power to apply to the wheels
                return dOutput;
            } else if (gnRobotSpin==CCW)  {
                //if got here navx will give a positive power to apply to the wheels
                return -dOutput;
            } else { //NOSPIN
                return 0;
            }
        }
        //if got here not working with a swerve
        return dOutput;
    }

    public double getRobotTargetToHold( ) {
        return gdYawAngleToHold;
    }


    /**
     * Setup the linear direction to move the robot from joystick or buttons
     *     Buttons-Robot oriented movement (called "crawling" below)
     *     Joystick- Feild oriented movement 
     *         What is a field oriented??
     *         Say the robot is pointing right as some random angle
     *         The direction stick is pressed forward
     *         The robot will move away from the driver and keep its random angle to the right
     *         The robot will NOT move right
     *    Crawl buttons:
     *         Move robot straight fwd,back,left,right NOT feild oriented (but still self correcting)
     * NOTE!!! setpoint NavX corrects the robot's orientation, not its direction.  
     *         Crawling involves straight movement only. In this case (and only this case) 
     *         setpoint NavX at the beginning of the crawl will help keep the robot straight
     *         by correcting the orientation of the robot as it tries to move straight
     * @param gamepad driver's gamepad, typically gamepad1
     * 
     * @return true if user requested a direction (by moving joystick or pressing button)
     *        sets gdRobotMoveFromDirCntlPolar, gnCrawling (needed for this method only)
     *
     */
    private void setRobotDirection(OpMode opMode,Gamepad gamepad) {
        int nCrawlSetting;
        //robot direction set by joystick
if(gbDebug) System.out.printf("setting direction...");
        if(chkDirectionStickPolar(gamepad)) { //chk the polar, field oriented direction the user wants to move
                                        //results stored in gdDirectionStickPolar if user move the stick
            //if got here, user moved the directional controlling joystick
            gdRobotMoveFromDirCntlPolar=gdDirectionStickPolar-getNavXYaw(); //this will move the robot in the field oriented direction
            gnCrawling=CRAWL_NONE; //needed for determining continuious crawl
            gdDirMagnitude=gdDirPwr; //get linear dir power magnitude from the throttle settings
            if(gbDebug) System.out.printf("direction stick pressed \n");
            return;
        }
        //user did not move directional joystick
        //check if robot direction set by crawl buttons
        //if pressed, need to capture when the user first pressed crawl button so can set the target yaw (setpointNavXYaw())
        //if got here, user did not move stick, so check if crawling
        if((nCrawlSetting=chkCrawlButtons(gamepad))!=CRAWL_NONE)  {
if(gbDebug) System.out.printf("crawl button pressed...");
            //crawl button pressed
            //check if this is a continuous crawl, or a new one
            opMode.telemetry.addData(gstrClassName,"CRAWL! %.2f",gdRobotMoveFromDirCntlPolar);
            opMode.telemetry.update();
            //not crawl none
            if(gnCrawling!=nCrawlSetting) { //NOTE gnCrawling could be CRAWL_NONE (before not crawling, but now crawling)
                opMode.telemetry.addData(gstrClassName,"NEW CRAWL! %.2f",gdRobotMoveFromDirCntlPolar);
                opMode.telemetry.update();
                //if got here, this is a new crawl
                setpointNavXYaw(getNavXYaw());
                if(gbDebug) System.out.printf("set crawl movement\n");
                switch(nCrawlSetting) {
                case CRAWL_LEFT:
                    gdRobotMoveFromDirCntlPolar = -90d;
                    break;
                case CRAWL_RIGHT:
                    gdRobotMoveFromDirCntlPolar = 90d;
                    break;
                case CRAWL_BACK:
                    gdRobotMoveFromDirCntlPolar = 180d;
                    break;
                case CRAWL_FWD:
                default:
                    gdRobotMoveFromDirCntlPolar = 0d;
                    break;
                }

                opMode.telemetry.addData(gstrClassName,"set gdRobotMove %.2f",gdRobotMoveFromDirCntlPolar);
                opMode.telemetry.update();
                gnCrawling=nCrawlSetting; //set gnCrawling for next use
                gdDirMagnitude=gdDirPwr;
                return;
            } else { //crawl button pressed, but same button as before
                if(gbDebug) System.out.printf("contiuious press ...");
                //if got here, this is a continous crawl
                gnCrawling=nCrawlSetting; //set gnCrawling for next use
                gdDirMagnitude=gdDirPwr;
                return;
            }
        } else { //nCrawlSetting==CRAWL_NONE
            //at this point, no crawl button pressed
            gnCrawling=nCrawlSetting; //set gnCrawling for next use
            gdDirMagnitude=0;//no robot direction requested by user
            //crawl buttons not pressed

        }
    }  //end of method



    /**
     * Check if stick is in dead zone
     *
     * @param gamepadToChk       driver's gamepad, typically gamepad1
     * @return true if stick in dead zone, false otherwise
     */
    private boolean chkStickDeadZone(Gamepad gamepad,boolean bControlStickToChk ) {

        //check if sticks are in dead zone
        ///CAUTION!!! Joysticks front/back are REVERSED!!!! left/right is okay-
        //     -          -
        //  -  L  +    -  R  +
        //     +          +
if(gbDebug) System.out.printf("Checking deadzone for %s\n",bControlStickToChk==ROT_STICK?"rotaton":"direction");
        
        if(bControlStickToChk == DIR_STICK) {
            if(gbDirectionStick==STICK_LEFT) {
                if (((gamepad.left_stick_x <= STICK_DEADZONE) && (gamepad.left_stick_x >= -STICK_DEADZONE)) &&
                        ((-gamepad.left_stick_y <= STICK_DEADZONE) && (-gamepad.left_stick_y >= -STICK_DEADZONE)))
                    return true;
                return false;  
            } else { //gbDirectionStick==STICK_RIGHT
                if (((gamepad.right_stick_x <= STICK_DEADZONE) && (gamepad.right_stick_x >= -STICK_DEADZONE)) &&
                        ((-gamepad.right_stick_y <= STICK_DEADZONE) && (-gamepad.right_stick_y >= -STICK_DEADZONE)))
                    return true;
                return false;
            }
        } else { //(bControlStickToChk == ROT_STICK) {
            if(gbRotationStick==STICK_LEFT) { //no need to check the y position
                if ((gamepad.left_stick_x <= STICK_DEADZONE) && (gamepad.left_stick_x >= -STICK_DEADZONE))
                    return true;
                return false;  
            } else { //gbRotationStick==STICK_RIGHT
                if ((gamepad.right_stick_x <= STICK_DEADZONE) && (gamepad.right_stick_x >= -STICK_DEADZONE)) 
                    return true;
                return false;
            }
        }
    }

    /**
     * Calculate the polar angle of the direction stick
     * sets global variable gdDirectionStickPolar
     * @param gamepad the gamepad object to check
     * @return true if direction stick was used, false otherwuse
     */
    private boolean chkDirectionStickPolar(Gamepad gamepad) {
        double dStickX, dStickY;
        double dAngle, dPolar;


        //calculate and correct stick's polar angle
        if (chkStickDeadZone(gamepad,DIR_STICK)) {
            //do not move robot in any direction from the direction stick
            //   robot may still spin from the yaw output
            gdDirectionStickPolar = 0;
            return false; //no linear direction
        }

        //if got here, the direction stick is pressed
        ///CAUTION!!! Joysticks front/back are REVERSED!!!! left/right is okay-
        //     -          -
        //  -  L  +    -  R  +
        //     +          +
        if (gbDirectionStick == STICK_LEFT) {
            //check if stick is in dead zone
            dStickX = (double) gamepad.left_stick_x;
            if (dStickX == 0) dStickX = .000001; //so don't divide by 0
            dStickY = -(double) gamepad.left_stick_y;
        } else { //STICK_RIGHT
            dStickX = (double) gamepad.right_stick_x;
            if (dStickX == 0) dStickX = .000001; //so don't divide by 0
            dStickY = -(double) gamepad.right_stick_y;
        }
//if(gbDebug) System.out.printf("   dStickX=%.2f dStickY=%.2f\n",dStickX,dStickY) ;
        dAngle = Math.toDegrees(Math.atan(dStickY / dStickX));
//if(gbDebug) System.out.printf("   dAngle=%.2f",dAngle) ;
        //Stick direction indicated by line
        //  The "0" indicates the angle calculated  above
        //  Must now convert the angle to a polar angle
        //
        //      | /          \ |              |              |
        //      |/ 0        -0\|              |              |
        //  ----+----      ----+----      ----+----      ----+----
        //      |              |           0 /|              |\ -0
        //      |              |            / |              | \
        //  polar angle=   polar angle=   polar angle=   polar angle=
        //  90-angle       -(angle+90)    -(angle+90)    -angle+90

       /*
       if((dStickX>=0)&&(dStickY>=0)) {
           dPolar=(double)90-dAngle;
       }
       else if((dStickX<0)&&(dStickY>=0)) {
           dPolar=-(dAngle+(double)90);
       }
       else if((dStickX<0)&&(dStickY<0)) {
           dPolar=-(dAngle+(double)90);
       }
       else if((dStickX>=0)&&(dStickY<0)) {
           dPolar=-dAngle+(double)90;
       }
       */
        //Note that the code below can be generalized
        //It is left in this if.then.else format to help reading and future debugging
        if (dStickX >= 0) {
            if (dStickY >= 0) {
                dPolar = (double) 90 - dAngle;
            } else { //(dStickY<0)
                dPolar = -dAngle + (double) 90;
            }
        } else { //dStickX<0
            if (dStickY >= 0) {
                dPolar = -(dAngle + (double) 90);
            } else { //(dStickY<0)
                dPolar = -(dAngle + (double) 90);
            }
        }
//if(gbDebug) System.out.printf("   dPolarAngle=%.2f\n",dPolar) ;

        gdDirectionStickPolar= dPolar;
        return true;
    }

    /**
     * set the crawl power for omni control  each mecanum wheel
     * "crawling" is moving the robot in a robot oriented direction fwd, back,lefi,right
     * why would you want to do this???
     * :  Say robot is at a 57 degree angle and the game element is directly in front
     * It is very difficult to move the direction stick at a 57 degree angle to approach the game element
     * Instead, the driver can simply press the crawl forward button to move the robot forward
     * The crawl buttons are above the direction control stick
     * If the gamepad left stick is the direction stick, the crawl buttons are dpad buttons
     * If the gamepad right stick is the direction stick, the crawl buttons are A,B,X,Y
     * Note that in omni mode, the buttons above the Rotation stick already rotates the robot
     *
     */
    private int chkCrawlButtons(Gamepad gamepad) {
       //The crawl buttons are above the control stick
       //   If the gamepad left stick is the direction stick, the crawl buttons are dpad buttons
       //   If the gamepad right stick is the direction stick, the crawl buttons are A,B,X,Y
       if (gbDirectionStick == STICK_RIGHT) {
           if (gamepad.y) {
               //apply pwr fwd
               return CRAWL_FWD ;
           } else if (gamepad.a) {
               //apply pwr back
               return CRAWL_BACK;
           } else if (gamepad.b) {
               //move right
               return CRAWL_RIGHT;
           } else if (gamepad.x) {
               //move left
               return CRAWL_LEFT;
           }
           return CRAWL_NONE; //shold not get here
       } else { //STICK_LEFT
           if (gamepad.dpad_up) {
               //apply pwr fwd
               return CRAWL_FWD ;
           } else if (gamepad.dpad_down) {
               return CRAWL_BACK ;
           } else if (gamepad.dpad_right) {
               return CRAWL_RIGHT ;
           } else if (gamepad.dpad_left) {
               return CRAWL_LEFT ;
           }
           return CRAWL_NONE; //shold not get here
       }
    }

    /**
     * Check robot throttle setting
     * Omni mode and Tank mode uses these throttle settings
     * Right Bumper sets throttle to max speed
     * Right Trigger sets throttle to min speed
     * Left Bumper increases speed up to max speed (unless disabled)
     * Left Trigger reduces speed down to min speed (unless disabled)
     *
     * @param gamepad driver's gamepad, typically gamepad1
     *                return: none.  global variables gdTankPwrMagnitude, gdTankMovement,gdOmniPwrMagnitude,gdMomentumAdjustment  set
     */
    private void chkThrottle(Gamepad gamepad) {
        if ((System.currentTimeMillis() - glLastControlModePress) < CONTROL_SENSITIVITY) return;
        if (gamepad.right_trigger > .5) {//hit right trigger, slow speed setting
            setThrottleSlow();
            glLastControlModePress = System.currentTimeMillis();
        } else if (gamepad.right_bumper) {//hit right bumper, fast speed setting
            setThrottleFast();
            glLastControlModePress = System.currentTimeMillis();
        }
    }
    /**
     *set the throttle to slow
     */
    public void setThrottleSlow() {
        gdDirPwr = gdSlowPwr;
        gdMomentumAdjustmentDeg=gdSlowMomentumAdjDeg;
        gdRotPwr = gdSlowPwr*gdRotPwrScale;
    }
    /**
     * Set the throttle to fast
     */ 
    public void setThrottleFast() {
        gdDirPwr = gdFastPwr;
        gdMomentumAdjustmentDeg=gdFastMomentumAdjDeg;
        gdRotPwr = gdFastPwr*gdRotPwrScale;
    }

    /**
     * Set all motors to a power
     * @param dPwr
     */
    public void setMotorPwr(double dPwr){
        swerve.pwrMotors(dPwr);
        
    }

    /**
     * Set motor power indvidually
      * @param dPwrLF
     * @param dPwrLB
     * @param dPwrRF
     * @param dPwrRB
     */
    public void setAllMotors(double dPwrLF, double dPwrLB, double dPwrRF, double dPwrRB){
        swerve.pwrMotorsAll(dPwrLF, dPwrLB, dPwrRF, dPwrRB);
    }

    /**
     * Calibrate the servo by looking at analog sensor voltage
     * @param linOpMode
     * @param nPos 0,90, -90 or 180
     * @param lCalibrationTime time to run and view calibration
     */
    public void calibrateServoLOp(LinearOpMode linOpMode,int nPos,long lCalibrationTime) {
        swerve.calibrateServoLOp(linOpMode,nPos,lCalibrationTime);
    }

    /**
     * crawl robot in poloar  direction
     * op mode, so need to call continuously
     * @param opMode opmode
     * @parm nCrawlDirection  CRAWL_LEFT, CRAWL_RIGHT...
     * @param dPwr power to apply
     */
    public void crawlRobot(OpMode opMode, int nCrawlDirection, double dPwr) {
        swerve.crawlRobot(opMode,nCrawlDirection,dPwr);
    }


    public boolean moveRobotPolar90LOp(LinearOpMode linopMode, long lTimeout, double dPwr) {
        //dont use this, sample only
        boolean bSuccess;

        bSuccess=swerve.rotateServoToPolar90LOp(linopMode,2000l);
        swerve.pwrMotors(dPwr);
        return bSuccess;
    }

    /**
     * Move robot in neg polar 90 direction
     * @param opMode
     * @param dPwr
     * @return
     */
    public boolean moveRobotNegPolar90LOp(LinearOpMode linopMode, long lTimeout, double dPwr) {
        //dont use this, test only
        boolean bSuccess;

        bSuccess=swerve.rotateServoToNegPolar90LOp(linopMode,2000l);
        swerve.pwrMotors(dPwr);
        return bSuccess;
    }

    /**
     * Move robot in polar 0 direction
     * @param opMode
     * @param dPwr
     * @return
     */
    public boolean moveRobotPolar0LOp(LinearOpMode linopMode,long lTimeout,double dPwr) {
        boolean bSuccess;
        //dont use this
        bSuccess=swerve.rotateServoToPolar0LOp(linopMode,2000l);
        swerve.pwrMotors(dPwr);
        return bSuccess;
    }

    /**
     * Move robot in polar 180 direction
     * @param opMode
     * @param dPwr
     * @return
     */
    /*
    public boolean moveRobotPolar180OP(OpMode opMode, double dPwr) {
        boolean bSuccess;

        bSuccess=swerve.rotateServoToPolar0Op(opMode); //axon servos cant' turn to 180
        if(bSuccess) {
            swerve.pwrMotors(-dPwr);
        }
        return bSuccess;
    }
     */
    /**
     * Check for user press of various control buttons
     * @param gamepad gamepad to check
     */
    private void chkControlOptions(Gamepad gamepad) {
        if ((System.currentTimeMillis() - glLastControlModePress) < CONTROL_SENSITIVITY) return;
        if (gamepad.left_stick_button) {
            gbDirectionStick = STICK_LEFT;
            gbRotationStick = STICK_RIGHT;
            glLastControlModePress = System.currentTimeMillis();
        }
        else if (gamepad.right_stick_button) {
            gbDirectionStick = STICK_RIGHT;
            gbRotationStick = STICK_LEFT;
            glLastControlModePress = System.currentTimeMillis();
        }
        if(NAVX_KL_LIB) {
            if(gamepad.guide) { //the big logitech button in the middle top of pad
                navX.zeroYaw();
            }
        }
    }

    /**
     *
     * @param linopMode Use linearop mode
     * @param nDistToMoveEncoderCount distance to move
     * @param lMaxTime timeout in ms
     * @param dRobotOrientationToMaintain orientation to maintain
     * @param nDir direction to move  AUTON_FWD,AUTON_RIGHT,AUTON_REV,AUTON_LEFT
     * @param dPwr power to apply
     * @return distance travelled in encoder count
     */

    public int autonmoveNavXLOp(LinearOpMode linopMode, int nDistToMove, long lMaxTime,
                                double dRobotOrientationToMaintain, int nDir, double dPwr) {
        long lTimeStamp = System.currentTimeMillis();
        double dPwrCalc = Math.abs(dPwr);
        boolean bExit = false;
        int nCurrentHorizPos = 0, nCurrentVertPos = 0;
        int nStartHorizPos = 0, nStartVertPos = 0;
        int nTargetHorizPos = 0, nTargetVertPos = 0;
        int nDistanceRemaining = 0, nDistanceTraveled = 0;
        String strBreakingHistory = "";
        double dDirectionalPolar=0, dDirectionalPwr=0;
        int nRC;
        double dCalc;

        //if (gnChassis == CHASSIS_SWERVE) {
             nStartHorizPos = swerve.getOdoHorizPos();
             nStartVertPos = swerve.getOdoVertPos();
        //} else {
        //    nStartHorizPos = mec.getOdoHorizPos();
        //     nStartVertPos = mec.getOdoVertPos();
        //}

        switch (nDir) {
            case AUTON_FWD:
                nTargetVertPos = nStartVertPos + nDistToMove;//since moving wheel fwd or right
                nTargetHorizPos = nStartHorizPos;
                dDirectionalPolar=0;
                break;
            case AUTON_RIGHT:
                nTargetVertPos = nStartVertPos;
                nTargetHorizPos = nStartHorizPos + nDistToMove;//mec moving wheel fwd or right
                dDirectionalPolar=90;
                break;
            case AUTON_REV:
                nTargetVertPos = nStartVertPos - nDistToMove;//since moving wheel backwards
                nTargetHorizPos = nStartHorizPos;
                dDirectionalPolar=180;

                break;
            case AUTON_LEFT:
                nTargetVertPos = nStartVertPos;
                nTargetHorizPos = nStartHorizPos + nDistToMove;//mec moving wheel backwards
                dDirectionalPolar=-90;
                break;
        }

        //set orientation
        setpointNavXYaw(dRobotOrientationToMaintain);

        while (linopMode.opModeIsActive() && !linopMode.isStopRequested() && !bExit) {
            if ((System.currentTimeMillis() - lTimeStamp) > lMaxTime) {
                linopMode.telemetry.addData(gstrClassName, "timeout");
                linopMode.telemetry.update();
                nRC = AUTON_TIMEOUT;
                bExit = true;
            }
            // if (gnChassis == CHASSIS_SWERVE) {
                 nCurrentHorizPos = swerve.getOdoHorizPos();
                 nCurrentVertPos = swerve.getOdoVertPos();
            // } else {
            //     nCurrentHorizPos = swerve.getOdoHorizPos();
            //     nCurrentVertPos = swerve.getOdoVertPos();
            // }
            switch (nDir) {
                case AUTON_FWD:
                    //since moving fwd or back
                    dDirectionalPwr = dPwr;
                    nDistanceTraveled = nCurrentVertPos-nStartVertPos;
                    if (nDistanceTraveled >= nDistToMove) {
                        dDirectionalPwr = 0;
                        bExit = true;
                    } else dDirectionalPwr = dPwr;

                    break;
                case AUTON_RIGHT:
                    //since moving wheel fwd or right
                    nDistanceTraveled = nCurrentHorizPos - nStartHorizPos;
                    if (nDistanceTraveled >= nDistToMove) {
                        dDirectionalPwr = 0;
                        bExit = true;
                    } else dDirectionalPwr = dPwr;
                    break;
                case AUTON_REV:
                    //since moving fwd or b
                    // start =50 current=30 dist=50-30=20
                    // target=-20  start =50 current=-10 dist=50-(-10)=60
                    nDistanceTraveled = nStartVertPos - nCurrentVertPos;
                    if (nDistanceTraveled >= nDistToMove) {
                        dDirectionalPwr = 0;
                        bExit = true;
                    } else dDirectionalPwr = dPwr;
                    break;
                case AUTON_LEFT:
                    //since moving wheel back or left
                    // target=-100 start =-10 current =-20
                    // target=50  start=100 current=90
                    // target=-20  start =50 current=30 dist=30-(-20)=50
                    // target=-20  start =50 current=-10 dist=-10-(-20)=10
                    // start =50 current=30 dist=50-30=20
                    // target=-20  start =50 current=-10 dist=50-(-10)=60
                    nDistanceTraveled =  nCurrentHorizPos-nStartHorizPos ;
                    if (nDistanceTraveled >= nDistToMove) {
                        bExit = true;
                        dDirectionalPwr = 0;
                    } else dDirectionalPwr = dPwr;
                    break;
            }
            //now get a correctional vector for movement
            dCalc = getNavXCorrectionPwr(linopMode);
            if (dCalc > 0) gnRobotSpin = CW;
            else if (dCalc < 0) gnRobotSpin=CCW;
            else gnRobotSpin = NOSPIN;
            gdRotMagnitude = Math.abs(dCalc);
            
            //now implment the movement
            if (gnChassis == CHASSIS_SWERVE) {
                swerve.operate(linopMode,dDirectionalPolar,dDirectionalPwr,gnRobotSpin,gdRotMagnitude);
            } else {
                mec.operate();
            }
    linopMode.telemetry.addData(gstrClassName,"Traveled:%d stopat:%d",nDistanceTraveled,nDistToMove);
    linopMode.telemetry.update();
            
        }//end while
        //need to stop robot
        swerve.operate(linopMode,dDirectionalPolar,0,NOSPIN,0);
   /*

        switch (nDir) {
            case AUTON_FWD:
                //since moving wheel fwd or right
                nDistanceRemaining = nTargetVertPos - nCurrentVertPos;
                nDistanceTraveled = nCurrentVertPos - nStartVertPos;
                break;
            case AUTON_RIGHT:
                //since moving wheel fwd or right
                nDistanceRemaining = nTargetCount - nCurrentPosHoriz;
                nDistanceTravelled = nCurrentPosHoriz - nStartCount;
                break;
            case AUTON_REV:
                nDistanceRemaining = nCurrentPosVert - nTargetCount;
                nDistanceTravelled = nStartCount - nCurrentPosVert;
                break;
            case AUTON_LEFT:
                nDistanceRemaining = nCurrentPosHoriz - nTargetCount;
                nDistanceTravelled = nStartCount - nCurrentPosHoriz;
                break;
        }
        //uncomment to check if still moving after breaking
        lTimeStamp = System.currentTimeMillis();
        while((linopMode.opModeIsActive())&&(System.currentTimeMillis()-lTimeStamp)<15000) {
            linopMode.telemetry.addData("Crab", "Final Orientation:%.2f Current %.2f",
                    dRobotOrientationToMaintain, getNavXYaw());
            linopMode.telemetry.addData("   ", "Rem:%d SlowAt:%d BreakAt:%d",
                    nDistanceRemaining,gnSlowDownDist,nBreakDist);
            linopMode.telemetry.addData("   ", "LF %.2f RF %.2f", gdResultantLFPwr, gdResultantRFPwr);
            linopMode.telemetry.addData("   ", "LB %.2f RB %.2f", gdResultantLBPwr, gdResultantRBPwr);

            linopMode.telemetry.addData("Crab", "Final Req:%d Dist:%d Rem:%d",nDistToMoveEncoderCount,nDistanceTravelled,nDistanceRemaining);
            linopMode.telemetry.addData("    ", "Raw Vert %d Horiz %d Start:%d",mtrLeftFront.getCurrentPosition(),
                    mtrLeftBack.getCurrentPosition(),nStartCount);
            linopMode.telemetry.addData("    ", "Diff: V:%d H:%d",
                    nDistToMoveEncoderCount-Math.abs(mtrLeftFront.getCurrentPosition()-nVertStart),
                    nDistToMoveEncoderCount-Math.abs(mtrLeftBack.getCurrentPosition()-nHorizStart));
            linopMode.telemetry.addData("    ", "BreakingHistory %s",strBreakingHistory);
            linopMode.telemetry.update();
        }

*/
        //now fine tune
        /*
        if ((nDir==AUTON_LEFT)||(nDir==AUTON_RIGHT)){
            if(Math.abs(nDistanceTravelled-nDistToMoveEncoderCount)>gnLinearAdjDist){
                autonmoveXPosLOp(linopMode,nTargetCount, 250l, 2000l,gdMinLinearPwr,gdMinLinearPwr,
                gdMinLinearPwr,gdMinLinearPwr);
            }
            if(Math.abs(nVertStart-mtrLeftFront.getCurrentPosition())>gnLinearAdjDist) {
                autonmoveYPosLOp(linopMode,nVertStart, 100l, 1000l,gdMinLinearPwr,gdMinLinearPwr,
                gdMinLinearPwr,gdMinLinearPwr);
            }

        }else if ((nDir==AUTON_FWD)||(nDir==AUTON_REV)) {
            if(Math.abs(nDistanceTravelled-nDistToMoveEncoderCount)>gnLinearAdjDist){
                autonmoveYPosLOp(linopMode,nTargetCount, 100l, 2000l,gdMinLinearPwr,gdMinLinearPwr,
                  gdMinLinearPwr,gdMinLinearPwr);
            }
            if(Math.abs(nHorizStart-mtrLeftBack.getCurrentPosition())>gnLinearAdjDist) {
                autonmoveXPosLOp(linopMode,nHorizStart, 100l, 1000l,gdMinLinearPwr,gdMinLinearPwr,
                gdMinLinearPwr,gdMinLinearPwr);
            }
        }
         */

        return(nDistanceTraveled);
    }

    /**
     * Move in preset direction
     * @param linopMode Use linearop mode
     * @param nDistToMoveEncoderCount distance to move
     * @param lMaxTime timeout in ms
     * @param nDir direction to move  AUTON_FWD,AUTON_RIGHT,AUTON_REV,AUTON_LEFT
     * @param dPwr power to apply
     * @return distance travelled in encoder count
     */
    public int autonmoveLOp(LinearOpMode linopMode, int nDistToMove, long lMaxTime,
                                int nDir, double dPwr) {
        long lTimeStamp = System.currentTimeMillis();
        double dPwrCalc = Math.abs(dPwr);
        boolean bExit = false;
        int nCurrentHorizPos = 0, nCurrentVertPos = 0;
        int nStartHorizPos = 0, nStartVertPos = 0;
        int nTargetHorizPos = 0, nTargetVertPos = 0;
        int nDistanceRemaining = 0, nDistanceTraveled = 0;
        String strBreakingHistory = "";
        double dDirectionalPolar = 0, dDirectionalPwr = 0;
        int nRC;
        double dCalc;


        //if (gnChassis == CHASSIS_SWERVE) {
        nStartHorizPos = swerve.getOdoHorizPos();
        nStartVertPos = swerve.getOdoVertPos();
        //} else {
        //    nStartHorizPos = mec.getOdoHorizPos();
        //     nStartVertPos = mec.getOdoVertPos();
        //}

        //setup servos
        switch (nDir) {
            case AUTON_FWD:
                swerve.rotateServoToPolar0LOp(linopMode,2000);
                break;
            case AUTON_LEFT:
                swerve.rotateServoToNegPolar90LOp(linopMode,2000);
                break;
            case AUTON_RIGHT:
                swerve.rotateServoToPolar90LOp(linopMode,2000);
                break;
            case AUTON_REV:
                swerve.rotateServoToPolar180LOp(linopMode,2000);
                break;
        }//end switch
        //set motor target positions
        switch (nDir) {
            case AUTON_FWD:
            case AUTON_LEFT:
            case AUTON_RIGHT:
            case AUTON_REV:
                nTargetVertPos = nStartVertPos + nDistToMove;//since moving wheel fwd
                nTargetHorizPos = nStartHorizPos;
                dDirectionalPolar = 0;
                swerve.setMotorRunToTargets(nDistToMove);
                break;
        }//end switch
        //run motors
        swerve.pwrMotors(dPwr);
        while (linopMode.opModeIsActive() && !linopMode.isStopRequested() && !bExit) {
            if ((System.currentTimeMillis() - lTimeStamp) > lMaxTime) {
                linopMode.telemetry.addData(gstrClassName, "timeout");
                linopMode.telemetry.update();
                nRC = AUTON_TIMEOUT;
                bExit = true;
            }
           if (!swerve.chkMotorsBusy()) bExit = true;
        }
        swerve.stopMotors();


        return 0;
    }//end class

    /**
     * calc the polor angle to correct movement.
     * mostly for debugging purposes
     * @param linopMode
     * @param dX X to correct.  Left is negative
     * @param dY Y to correct
     * @return
     */
    public double autoncalcPolarXYReqLOp(LinearOpMode linopMode, double dX, double dY) {
        return swerve.calcPolarXYReq(dX,dY);
    }

    /**
     * rotate servos to the polor angle to correct movement
     * mostly for debugging purposes
     * @param linopMode
     * @param dX X to correct.  Left is negative
     * @param dY Y to correct
     * @return
     */
    public boolean autonRotateServoXYReqLOp(LinearOpMode linopMode, double dX, double dY,long lTimeout) {
        return swerve.rotateServoToXYReq(linopMode,dX,dY,lTimeout);
    }

    /**
     * move to XY request until Y stop or timeout
     * @param linopMode
     * @param dX X req dist.  left is neg
     * @param dY Y req dist
     * @param dPwr pwr to move
     * @param dStopDist distance to stop
     * @param lTimeout timeout
     * @return false when done with move, true while moving
     */
    public boolean autonmoveXYReqOp(LinearOpMode linopMode, double dX, double dY,double dPwr,
                                     double dStopDist,long lTimeToAttempt) {
        if((linopMode.opModeIsActive())&&(dY>dStopDist)) {
            swerve.rotateServoToXYReq(linopMode,dX,dY,lTimeToAttempt);
            swerve.pwrMotors(dPwr);
        } else {
            swerve.stopMotors();
            /*debugging below */
            linopMode.telemetry.addData(gstrClassName,"autonmoveXYReqOp STOP");
            linopMode.telemetry.update();
            linopMode.sleep(5000);

            /* */
        }
        if(dY>dStopDist) return true;
        return false;
    }


    /**
     * Move in preset direction
     * @param linopMode Use linearop mode
     * @param nDistToMoveEncoderCount distance to move
     * @param lMaxTime timeout in ms
     * @param nDir direction to move  AUTON_FWD,AUTON_RIGHT,AUTON_REV,AUTON_LEFT
     * @param dPwr power to apply
     * @return distance travelled in encoder count
     */
    public int autonmoveAngleLOp(LinearOpMode linopMode, int nDistToMove,int nTolerance, long lMaxTime,
                           double dAngle, double dPwr) {
        long lTimeStamp = System.currentTimeMillis();
        double dPwrCalc = Math.abs(dPwr);
        boolean bExit = false;
        int nCurrentHorizPos = 0, nCurrentVertPos = 0;
        int nStartHorizPos = 0, nStartVertPos = 0;
        int nTargetHorizPos = 0, nTargetVertPos = 0;
        int nDistanceRemaining = 0, nDistanceTraveled = 0;
        String strBreakingHistory = "";
        double dDirectionalPolar = 0, dDirectionalPwr = 0;
        int nRC;
        double dCalc;

        //if (gnChassis == CHASSIS_SWERVE) {
        nStartHorizPos = swerve.getOdoHorizPos();
        nStartVertPos = swerve.getOdoVertPos();
        //} else {
        //    nStartHorizPos = mec.getOdoHorizPos();
        //     nStartVertPos = mec.getOdoVertPos();
        //}

        //setup servos
        swerve.rotateServoToPosLOp(linopMode,dAngle,2000);

        //set motor target positions
        swerve.setMotorRunToTargets(nDistToMove);
        //run motors
        swerve.pwrMotors(dPwr);
        while (linopMode.opModeIsActive() && !linopMode.isStopRequested() && !bExit) {
            if ((System.currentTimeMillis() - lTimeStamp) > lMaxTime) {
                linopMode.telemetry.addData(gstrClassName, "autonmoveAngleLOP timeout");
                linopMode.telemetry.update();
                nRC = AUTON_TIMEOUT;
                bExit = true;
            }
            //if (!swerve.chkMotorsBusy()) bExit = true;
            if(swerve.chkMotorsReachedPosLOp(linopMode,nTolerance,lMaxTime)) bExit=true;
            //linopMode.telemetry.addData(gstrClassName,"autonmoveangle:exit %s",bExit);
            //linopMode.telemetry.update();
        }
        swerve.stopMotors();


        return 0;
    }//end class

    public void autonfinetuneYawLOp(LinearOpMode linopMode,int nDir,double dPwr,int nDist,
                                 float fTolerancePct,long lTimeout) {
        int nTolerance=Math.abs(Math.round((float)nDist*fTolerancePct));
        swerve.rotateServoToPolar0Op(linopMode);
        if(nDir==CW) {
            swerve.setMotorSpinToTargets(nDist);
        } else if (nDir==CCW) {
            swerve.setMotorSpinToTargets(-nDist);
        }
        swerve.pwrMotors(dPwr);
        swerve.chkMotorsReachedPosLOp(linopMode,nTolerance,lTimeout);
        swerve.stopMotors();
    }

    /**
     * spin in preset direction
     * @param linopMode Use linearop mode
     * @param nDistToSpin distance to spin, pos value is CW, neg is CCW
     * @param lMaxTime timeout in ms
     * @param dPwr power to apply
     *             //move right side back, leave left locked
     * @return  true if spun to distance within tolerance
     */
    public boolean autonspinDistLOp(LinearOpMode linopMode, int nDistToSpin, int nTolerance, long lMaxTime,double dPwr) {
        long lTimeStamp = System.currentTimeMillis();
        double dPwrCalc = Math.abs(dPwr);
        boolean bExit = false;

        //setup servos
        swerve.rotateServoToSpinPosLOp(linopMode,1000);

        //set motor target positions
        swerve.setMotorSpinToTargets(nDistToSpin);
        //run motors
        swerve.pwrMotors(dPwr);
        while (linopMode.opModeIsActive() && !linopMode.isStopRequested() && !bExit) {
            if ((System.currentTimeMillis() - lTimeStamp) > lMaxTime) {
                linopMode.telemetry.addData(gstrClassName, "autonspinDistLOP timeout");
                linopMode.telemetry.update();
                bExit = true;
            }
            //if (!swerve.chkMotorsBusy()) bExit = true;
            if(swerve.chkMotorsReachedPosLOp(linopMode,nTolerance,lMaxTime)) bExit=true;
            //linopMode.telemetry.addData(gstrClassName,"autonmoveangle:exit %s",bExit);
            //linopMode.telemetry.update();
        }
        swerve.stopMotors();

        if(bExit) return false;
        else return true;
    }//end class

    /**
     * Adjust the linear power by slowing down and approach target
     * @param dLinearPwr "Normal" linear power to return
     * @param nDistRemaining Distance remaining to target
     * @param nSlowDownDist Distance to target to start braking
     * @param nBrakeDist
     * @param dDIYLinearP
     * @param dMinLinearPwr
     * @return
     */
    private double adjustLinearPwr(double dLinearPwr, int nDistRemaining,int nSlowDownDist,
                                   int nBrakeDist,double dDIYLinearP, double dMinLinearPwr){
        double dCalcPwr=0d;

        //check if need to slowdown
        if(nDistRemaining>nSlowDownDist) return dLinearPwr;
        //if got here, need to start slowing down
        //         SlowDown  BrakeDist  Target
        //+--------+---------+----------+
        //        1000       100
        //
        if((nDistRemaining<=nSlowDownDist)&&(nDistRemaining>nBrakeDist)){
            //need number that gets smaller as DistRemain gets smaller
            dCalcPwr=(((double)nDistRemaining-(double)nBrakeDist)/((double)nSlowDownDist-(double)nBrakeDist))*dDIYLinearP*dLinearPwr;
            //eX distremain=1000    (1000-100)/(1000-100) 1
            //eX distremain=300    (300-100)/(1000-100) 200/900
            //eX distremain=101    (101-100)/(1000-100) 1/900
            if(dCalcPwr<dMinLinearPwr) return dMinLinearPwr;
            return dCalcPwr;
        }

        return dMinLinearPwr;

    }
    /**
     * Auton pivot turn (call once) using NavX
     *   PID controls the power of the turn
     *   Adjust P for increasing/decreasing the rate
     *
     * @param linopMode Must use linear op mode
     * @param dTargetHeading Pivot to this heading (smallest path, CCW or CW)
     * @param lMaxTime_msec Timeoutn milli seconds
     * @return DONE, TIMEDOUT, INTERRUPTED
     */
    public int autonpivotTurnNavXLOp(LinearOpMode linopMode, double dTargetHeading, long lMaxTime_msec) {
        long lTimeStamp = System.currentTimeMillis();
        boolean bExit=false;;
        double dInit=999,dDistToRotate;
        double dPwr;


        //correct input
        if (dTargetHeading >= 0) { //positive target heading
            if (dTargetHeading > 360) dTargetHeading %= 360; //make target less than 360
            if (dTargetHeading > 180) {//target is too positive, make it negative
                //target is between 180 and 360o
                dTargetHeading = dTargetHeading - 360;
            }
        } else { //negative target heading
            if (dTargetHeading < -360) dTargetHeading %= 360; //make target less than -360
            if (dTargetHeading < -180) {//target is too negative, make it positive
                dTargetHeading = dTargetHeading + 360;
            }
        }
        //check if within tolerance
        dDistToRotate = calcDistToRotateDegrees(dTargetHeading,getNavXYaw());
        if(Math.abs(dDistToRotate)<gdToleranceDeg) return 0;

        //sweep past target
        dInit=dDistToRotate;
        setpointNavXYaw(dTargetHeading);
        swerve.rotateServoToSpinPosLOp(linopMode,1000);

        while (linopMode.opModeIsActive() && !linopMode.isStopRequested() &&
                !Thread.currentThread().isInterrupted() && !bExit) {
            if ((System.currentTimeMillis() - lTimeStamp) > lMaxTime_msec) {
                linopMode.telemetry.addData(gstrClassName, "pivot timeout:target:%.2f actual:%.2f", dTargetHeading, getNavXYaw());
                linopMode.telemetry.update();
                return AUTON_TIMEOUT;
            }
            dDistToRotate = calcDistToRotateDegrees(dTargetHeading,getNavXYaw());
            //check if swept past target
            //get the spin direction
            if (dDistToRotate > 0) { //need to spin CW
                if(dInit<0) {//initial spin was CCW
                    //if got here, swept past;
                    bExit=true;
                    continue; //exit while
                }
                //if got here need to move in same direction
                gnRobotSpin = CW;
            } else if (dDistToRotate < 0) { //need to spin CCW
                if(dInit>0) {//initial spin was CW
                    //if got here, swept past;
                    bExit=true;
                    continue; //exit while
                }
                //if got here need to move in same direction
                gnRobotSpin=CCW;
            }
            else gnRobotSpin = NOSPIN;

            gdRotMagnitude=getNavXCorrectionPwr(linopMode);
            linopMode.telemetry.addData(gstrClassName,"pivot Correction:%.2f",gdRotMagnitude);
            
            //linopMode.sleep(2000);
            if (gnChassis == CHASSIS_SWERVE) {
                
                dPwr=gdRotMagnitude;
                linopMode.telemetry.addData(gstrClassName,"swerve power:%.2f spin:%d",dPwr,gnRobotSpin);
                
                if(gnRobotSpin==CW) {
                    swerve.pwrMotorsAll(dPwr,dPwr,-dPwr,-dPwr);
                } else if(gnRobotSpin==CCW) {
                    swerve.pwrMotorsAll(-dPwr,-dPwr,dPwr,dPwr);
                } else {
                    swerve.pwrMotors(0);
                }
                //already done in getNavXCorrection gdRotMagnitude = Math.abs(dCalc);
                // swerve.operate(linopMode,0d,0d,gnRobotSpin,gdRotMagnitude);
            } else {
                mec.operate();
            }
            

            //if(Math.abs(dDistToRotate)<gdToleranceDeg) bExit=true;

linopMode.telemetry.addData(gstrClassName, "Target:%.2f Current %.2f", dTargetHeading, getNavXYaw());
linopMode.telemetry.update();
        }


        return 0;
    }


    /**
     * init Kauai Labs navX
     * navX must be named navx in robot configuration
     *
     * @param opMode      used to find navx in robot configuration
     * @param strNavXName name of navX
     *                    return: none. Sets global variable gnNavXStatus NAVX_QC_LIB or NAMX_NONE
     *                    NAVX_KAUAILABS_LIB  Use Kauai Labs Libraries for orientation
     *                    NAVX_QC_LIB  Use FTC/Qualcomm libraries for orientation
     *                    NAVX_NONE no navx found
     */
    private boolean initNavX(OpMode opMode, String strNavXName) {
        //setup nav
        if(NAVX_KL_LIB)
           opMode.telemetry.addData(gstrClassName, "   KL " + strNavXName);
        else
            opMode.telemetry.addData(gstrClassName, "   QC " + strNavXName);


        if(NAVX_KL_LIB) {
           if (initNavXKL(opMode, strNavXName) == true) {
               //Kauai Labs libraries
               gnNavXStatus = NAVX_KAUAILABS_LIB;
               return true;
           }
        }

        //if got here, no KL lib, or KL init failed
        //Using FTC/Qualcomm  libraries
        if (initNavXQC(opMode, strNavXName) != NAVX_QC_LIB) {
             gnNavXStatus = NAVX_ERROR;
             return true;
        } else { //calibration ok
             gnNavXStatus = NAVX_QC_LIB;
             return false;
        }



    }


    /**
     *Initiate the Kauai Labs NavX Library 
     * 
     * @param opMode       Opmode to use
     * @param strNavXName  Name of NaxX defined in robot controller
     * @return
     */
    private boolean initNavXKL0(OpMode opMode, String strNavXName) {
        //instantiating class auto starts navigation
        //double YAW_PID_P = 0.005;
        double YAW_PID_P = gdYAW_P;
        double YAW_PID_I =4d;
        double YAW_PID_D = 0.0;
        int DEVICE_TIMEOUT_MS = 500;
        boolean bCalibrationComplete = false;
        long lStartTime = System.currentTimeMillis();

        navX = getInstance(opMode.hardwareMap.get(NavxMicroNavigationSensor.class, strNavXName),
                AHRS.DeviceDataType.kProcessedData, NAVX_DEVICE_UPDATE_RATE_HZ);
        // Create a PID Controller which uses the Yaw Angle as input.
        yawPIDController = new navXPIDController( navX,
                navXPIDController.navXTimestampedDataSource.YAW);


        yawPIDController.setSetpoint(90d);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        //yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, gdToleranceDeg);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        //navX.zeroYaw();
        //yawPIDResult = new navXPIDController.PIDResult();

        opMode.telemetry.addData("navX-Micro", "Startup Calibration");
        while ((!bCalibrationComplete) && ((System.currentTimeMillis() - lStartTime) < 30000)) {
            // navX-Micro Calibration completes automatically 15 seconds after it is
            //powered on, as long as the device is still.  To handle the case where the
            //navX-Micro has not been able to calibrate successfully, hold off using
            //the navX-Micro Yaw value until calibration is complete.

            bCalibrationComplete = !navX.isCalibrating();
        }
        opMode.telemetry.addData("navX-Micro", "calibration %s", bCalibrationComplete ? "complete" : "error");
        if (!bCalibrationComplete) return false;

        //test below...
        navX.zeroYaw();
        try {
            yawPIDController.enable(true);

        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */
            final double TOTAL_RUN_TIME_SECONDS = 30.0;
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            lStartTime=System.currentTimeMillis();
            while ( ((System.currentTimeMillis()-lStartTime) < (TOTAL_RUN_TIME_SECONDS*1000)) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        opMode.telemetry.addData(gstrClassName,"On Target");
                    } else {
                        double output = yawPIDResult.getOutput();
                        opMode.telemetry.addData(gstrClassName,"Not on target PIDOutput:%.2f",output );
                    }
                } else {
                    /* A timeout occurred */
                    opMode.telemetry.addData(gstrClassName,"timeout" );
                    //Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
                opMode.telemetry.addData(gstrClassName,"Yaw:%.2f",navX.getYaw());
                opMode.telemetry.update();
            }
            opMode.telemetry.addData(gstrClassName, "NavX KL init test");
        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
            opMode.telemetry.addData(gstrClassName, "NavX KL Init Exception caught");
        }
        finally {
            //navx_device.close();
            opMode.telemetry.addData(gstrClassName, "NavX KL Init complete");
        }
        return true;
    }
    private boolean initNavXKLInitTest(OpMode opMode, String strNavXName) {
        final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

        final double TARGET_ANGLE_DEGREES = 90.0;
        final double TOLERANCE_DEGREES = 2.0;
        final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
        final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
        final double YAW_PID_P = 0.005;
        final double YAW_PID_I = 0.0;
        final double YAW_PID_D = 0.0;

        boolean calibration_complete = false;
        long lTimestamp;
        navX = AHRS.getInstance(opMode.hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);


        //Create a PID Controller which uses the Yaw Angle as input.
        yawPIDController = new navXPIDController( navX,
                navXPIDController.navXTimestampedDataSource.YAW);

        // Configure the PID controller
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);


        while ( !calibration_complete ) {
            //navX-Micro Calibration completes automatically ~15 seconds after it is
            //powered on, as long as the device is still.  To handle the case where the
            //navX-Micro has not been able to calibrate successfully, hold off using
            //the navX-Micro Yaw value until calibration is complete.

            calibration_complete = !navX.isCalibrating();
            if (!calibration_complete) {
                opMode.telemetry.addData(gstrClassName, "NavX Startup Calibration in Progress");
                opMode.telemetry.update();
            }
        }
        navX.zeroYaw();
        yawPIDController.enable(true); //new placement
        try {
            //yawPIDController.enable(true);

            //Wait for new Yaw PID output values, then update the motors
            // with the new PID value with each new output value.

            final double TOTAL_RUN_TIME_SECONDS = 30.0;
            int DEVICE_TIMEOUT_MS = 500;
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            lTimestamp=System.currentTimeMillis();
            while ( ((System.currentTimeMillis()-lTimestamp) < (TOTAL_RUN_TIME_SECONDS*1000)) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        opMode.telemetry.addData(gstrClassName,"on target");
                    } else {
                        double output = yawPIDResult.getOutput();
                        opMode.telemetry.addData(gstrClassName,"not on target output=%.2f",output);
                    }
                } else {
                    /* A timeout occurred */
                    //Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                    opMode.telemetry.addData(gstrClassName,"timeout");
                }
                opMode.telemetry.addData(gstrClassName,"Yaw:%.2f",navX.getYaw());
                opMode.telemetry.update();
            }
        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        finally {
            //navx_device.close();
            opMode.telemetry.addData(gstrClassName,"NavX test Complete");
        }
        return true;
    }
    private boolean initNavXKL(OpMode opMode, String strNavXName) {
        //copied (with some mods) from KL sample code
        final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

        final double TARGET_ANGLE_DEGREES = 90.0;
        final double TOLERANCE_DEGREES = 2.0;
        final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
        final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
        final double YAW_PID_P = .008d;//0.005;
        final double YAW_PID_I = 0.0; //0.0;
        final double YAW_PID_D = 0.0;

        boolean calibration_complete = false;
        long lTimestamp;
        navX = AHRS.getInstance(opMode.hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
                
opMode.telemetry.addData(gstrClassName,"YAW PID P set to %.2f", YAW_PID_P);
opMode.telemetry.update();


        //Create a PID Controller which uses the Yaw Angle as input.
        yawPIDController = new navXPIDController( navX,
                navXPIDController.navXTimestampedDataSource.YAW);

        // Configure the PID controller
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);


        while ( !calibration_complete ) {
            //navX-Micro Calibration completes automatically ~15 seconds after it is
            //powered on, as long as the device is still.  To handle the case where the
            //navX-Micro has not been able to calibrate successfully, hold off using
            //the navX-Micro Yaw value until calibration is complete.

            calibration_complete = !navX.isCalibrating();
            if (!calibration_complete) {
                opMode.telemetry.addData(gstrClassName, "NavX Startup Calibration in Progress");
                opMode.telemetry.update();
            }
        }
        opMode.telemetry.addData(gstrClassName, "NavX Calibration Complete");
        opMode.telemetry.update();
        opMode.telemetry.addData(gstrClassName, "NavX Calibration Complete");//so displays again
        navX.zeroYaw();
        yawPIDController.enable(true); //new placement
        if(calibration_complete) return true;
        return false ;

        /*
        try {
            //yawPIDController.enable(true);

            //Wait for new Yaw PID output values, then update the motors
            // with the new PID value with each new output value.

            final double TOTAL_RUN_TIME_SECONDS = 30.0;
            int DEVICE_TIMEOUT_MS = 500;
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            lTimestamp=System.currentTimeMillis();
            while ( ((System.currentTimeMillis()-lTimestamp) < (TOTAL_RUN_TIME_SECONDS*1000)) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        opMode.telemetry.addData(gstrClassName,"on target");
                    } else {
                        double output = yawPIDResult.getOutput();
                        opMode.telemetry.addData(gstrClassName,"not on target output=%.2f",output);
                    }
                } else {
                    //A timeout occurred
                    //Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                    opMode.telemetry.addData(gstrClassName,"timeout");
                }
                opMode.telemetry.addData(gstrClassName,"Yaw:%.2f",navX.getYaw());
                opMode.telemetry.update();
            }
        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        finally {
            //navx_device.close();
            opMode.telemetry.addData(gstrClassName,"NavX test Complete");
        }
        return true;
        */
    }
    /**
     * Calibrate navX using QualComm libraries
     *
     * @param opMode      opmode calling this method
     * @param strNavXName name of navX in robot configuration
     * @return NAVX_QC_LIB navX using QualComm Libraries, NAVX_ERROR problem configuring navX, NVX_NONE cannot find navX,
     * global variable gyro set
     */
    private int initNavXQC(OpMode opMode, String strNavXName) {
        NavxMicroNavigationSensor navxMicro;
        try {
            long lTimeStart = System.currentTimeMillis();
            boolean bTimeout = false;

            navxMicro = opMode.hardwareMap.get(NavxMicroNavigationSensor.class, strNavXName);
            gyro = (IntegratingGyroscope) navxMicro;

            // The gyro automatically starts calibrating. This takes a few seconds.
            opMode.telemetry.addData("Gyro:", "Calibrating. Do Not Move!");

            // Wait until the gyro calibration is complete
            while (navxMicro.isCalibrating() && !bTimeout) {
                opMode.telemetry.addData("calibrating", "...");
                if ((System.currentTimeMillis() - lTimeStart) < 10000)
                    bTimeout = true; //10 second timeout
                Thread.sleep(50);
            }
            if (bTimeout) return NAVX_ERROR;
            return NAVX_QC_LIB;
        } catch (IllegalArgumentException e) {
            opMode.telemetry.addData("argument", "%s", e.toString());
            //Thread.currentThread().interrupt();
            return NAVX_NONE;
        } catch (InterruptedException e) {
            opMode.telemetry.addData("interrupt", "%s", e.toString());
            //Thread.currentThread().interrupt();
            return NAVX_ERROR;
        } catch (InternalError e) {
            opMode.telemetry.addData("internal error", "%s", e.toString());
            //Thread.currentThread().interrupt();
            return NAVX_ERROR;
        } catch (Exception e) {
            opMode.telemetry.addData("internal error", "%s", e.toString());
            //Thread.currentThread().interrupt();
            return NAVX_ERROR;
        }
    }

    /**
     * enable the navX
     * Only change state if necessary
     *
     * @param bEnable
     */
    private void enableNavXYawPID(boolean bEnable) {
        /*
        if(gnNavXStatus==NAVX_KAUAILABS_LIB) {
            if(bEnable==true) { //want to enable the yawPIDController
                if(!yawPIDController.isEnabled()) yawPIDController.enable(bEnable);
            }
        }
        */

        if (gnNavXStatus == NAVX_KAUAILABS_LIB) {
            if (bEnable == true) { //want to enable the yawPIDController
                if (!yawPIDController.isEnabled()) yawPIDController.enable(bEnable);
            } else { //want to disable yawPIDController
                 if (yawPIDController.isEnabled()) yawPIDController.enable(bEnable);
            }
        }
        //no equivalent in FIRST library
     }

    /**
     * Reset the motors for teleop
     *
     //* @param bUseEncoders true=use encoders, false= don't use encoders
     */

    public void resetTeleop(boolean bUseEncoders) {
        swerve.resetTeleop(bUseEncoders);
    }

    public void resetAuton(boolean bUseEncoders) {
        swerve.resetAuton(bUseEncoders);
    }


    /**
     * Close the chassis
     *
     * @param opMode opmode calling this method
     */
    public void shutdown(OpMode opMode) {
        if (isNavXConnected()) enableNavXYawPID(false);
        closeNavX();
    }
    /**
     * Close navX
     */
    private void closeNavX() {
        if (gnNavXStatus == NAVX_KAUAILABS_LIB) {

            if (yawPIDController.isEnabled()) yawPIDController.enable(false);
            yawPIDController.close();
            navX.close();


        } else {
            //nothing equivalent in gyro classes
        }
    }

    /**
     * Check if navX is connected
     *
     * @return NavXConnection stauts
     */

    public boolean isNavXConnected() {
        if (gnNavXStatus == NAVX_KAUAILABS_LIB) {
            return (navX.isConnected());
        } else {
            //no equivalent without dim, so send true an pray
            return true;
        }
    }

    /** 
    * Set the robot orientation (yaw angle) to hold
    * The robot will try to hold this angle
    * The angle to hold is a polor angle. The 0 angle is where the navX calibration was made
    * in initNavX
    *
    * @param dAngleToSet the polar angle to hold -180-0-180
    */
    public void setpointNavXYaw(double dAngleToSet) {
        if (gnNavXStatus == NAVX_KAUAILABS_LIB) {
           yawPIDController.setSetpoint(dAngleToSet);
        } else {
           setDIYSetPoint(dAngleToSet);
        }
        //must set dYawAngleToHold regardless
        gdYawAngleToHold = dAngleToSet;
    }


     /**
     * DIY way of calculating the rotate to angle correction power
     * <p>
     * The returned value is meant to add to the left wheels (and subtract from the right wheels)
     * <p>
     * uses global variable  glCrudeYawCorrectionLast;
     *
     * @return the correction power between -1d and 1d
     */
    private double calcDIYYawCorrectionPwr(OpMode opMode) {
        double dTargetPolar = gdYawAngleToHold;
        double dCurrentPolar = getNavXYaw();

        double dDistToRotate,dDistToRotateAbs;
        double dDIYYawPwr=0;

        //find out how much need to correct, and in what direction
        dDistToRotate=calcDistToRotateDegrees(dTargetPolar, dCurrentPolar);
if(gbDebug) System.out.printf("%s:Target:%.2f Current%.2f\n",gstrClassName,dTargetPolar,dCurrentPolar);
        dDistToRotateAbs = Math.abs(dDistToRotate);
        //opMode.telemetry.addData(gstrClassName,"DIY DistToRotate:%.2f",dDistToRotate);
        
        //At this point, have the shortest degrees to rotate, and the direction
        //can now calculate the yaw correctional power
        //check if within tolerance.. dDegToRotate is a magnitude, TOLERANCE is a magnitude
        if (dDistToRotateAbs < gdToleranceDeg) {
            //opMode.telemetry.addData(gstrClassName,"DIY On Target",dDistToRotate);
            //telemetry.update();
            //within limits
            return 0d;
        }

        //Calculate the P, or proportional power.  The larger the degress to rotate,
        //   the larger the P
        dDIYYawPwr =  dDistToRotate/180d * gdYAW_P *gdRotPwr; //dDistToRotate will be between 0 and 180
                                                       //divide by 180 so value is always under 1

if(gbDebug) System.out.printf("%s:DIY YawPwr dDistToRotate/180 * gdDIYYaw_P * gdRotPwr =%.2f \n",gstrClassName,dDIYYawPwr);
if(gbDebug) System.out.printf("   dDistToRotate/180 = %.2f/180d = %.2f\n",dDistToRotate,dDistToRotate/180d);
if(gbDebug) System.out.printf("   gdDIYYaw_P=%.2f gdRotPwr=%.2f\n", gdYAW_P,gdRotPwr);
                                                       
        //recall that the gdMinYawPwr is a magnitude
        if(dDIYYawPwr<0) { //CCW correction
            if(-dDIYYawPwr<gdMinYawPwr) //CCW power too low
                dDIYYawPwr=-gdMinYawPwr;
            else if(-dDIYYawPwr>gdRotPwr) //CCW power too high
                dDIYYawPwr=-gdRotPwr;
        
        } else { //CW correction dDIYYawPwr>0;
            if(dDIYYawPwr<gdMinYawPwr) //CW power too low
                dDIYYawPwr=gdMinYawPwr;
            else if(dDIYYawPwr>gdRotPwr) //CCW power too high
                dDIYYawPwr=gdRotPwr;
            
        }
if(gbDebug) System.out.printf("   after cap dDIYYawPwr=%.2f\n",dDIYYawPwr);
        
        
        //opMode.telemetry.addData(gstrClassName,"DIY Pwr:%.2f",dDIYYawPwr);
        //opMode.telemetry.update();
        
        return(dDIYYawPwr);
    }
    /**
     * Calculate the smallest distance between the target and current robot rotation angle
     *
     * @param dTargetPolar  Polar angle to target
     * @param dCurrentPolar Current polar angle
     * @return magnitude of the smallest distance between target and current polar angle. Positive if need to rotate clockwise to travel smallest distance, negative if need to rotate counterclockwise
     */
    private double calcDistToRotateDegrees(double dTargetPolar, double dCurrentPolar) {
        double dDegToRotate = 0;
        int nRotationDirection = 0;
        double dTstCCW, dTstCW;

        dCurrentPolar = fixPolar(dCurrentPolar);
        dTargetPolar = fixPolar(dTargetPolar);
        //first, find the distance need to rotate for correction in polar degree
        //   calculate the distance to rotate CW
        //   calculate the distance to rotate CCW,
        //   use the smaller distance
        //
        //Sample calculations
        //    Current    Target    Correction           Dir
        //    --------   ------    -----------------
        //    0 180    >  0 180    abs(target-current)  CCW
        //    0 180    <  0 180    abs(target-current)   CW
        //    0 180      -180 0    current+abs(target)  CCW
        //                         (180-current) + abs(-180-target)  CW
        //   -180 0    > -180 0    abx(target-current)  CCW
        //   -180 0    < -180 0    abx(target-current)  CW
        //   -180 0       0 180    abs(current)+target   CW
        //                0 180    (-180-current)+ (180-target) CCW
        if ((dCurrentPolar >= 0) && (dCurrentPolar <= 180)) {
            if ((dTargetPolar >= 0) && (dTargetPolar <= 180)) {
                //    Current    Target    Correction           Dir
                //    --------   ------    -----------------
                //    0 180    >  0 180    abs(target-current)  CCW
                //    0 180    <  0 180    abs(target-current)   CW
                if (dTargetPolar <= dCurrentPolar) {
                    //         |  T
                    //         |    C
                    //         |
                    // - - - - + - - - -
                    //if here need to move counter clockwise
                    dDegToRotate = dCurrentPolar - dTargetPolar; //make dDegToRotate pos
                    if (dDegToRotate == 0) nRotationDirection = 0;
                    else nRotationDirection = -1; //Counter clockwise movement
                } else {  //dTargetPolar>dCurrentPolar
                    //         |  C
                    //         |    T
                    //         |
                    // - - - - + - - - -
                    //         |
                    //         |    C
                    //         |  T
                    //if here need to move clockwise
                    dDegToRotate = dTargetPolar - dCurrentPolar; //make dDegToRotate pos
                    nRotationDirection = 1; //Clockwise movement
                }
                //at this point, recall that this is still the condition: dCurrentPolar>=0 && dCurrentPolar<=180
            } else { //dTargetPolar<0 && dTargetPolar <= -180
                //find shortest path
                //         |  C1
                //     T1  |   C3
                //    T2   |
                // - - - - + - - - -
                //    T3   |   C2
                //     T4  |  C4
                //    Current    Target    Correction           Dir
                //    --------   ------    -----------------
                //    0 180      -180 0    current+abs(target)  CCW
                //                         (180-current) + abs(-180-target)  CW
                //Check dist for a CCW turn
                dTstCCW = dCurrentPolar + Math.abs(dTargetPolar);
                //Check dist for a CC turn
                dTstCW = (180d - dCurrentPolar) + (180d + dTargetPolar);
                if (dTstCCW >= dTstCW) {
                    //CW is shorter path
                    dDegToRotate = dTstCW;
                    if (dDegToRotate == 0) nRotationDirection = 0;
                    nRotationDirection = 1; //Clockwise movement
                } else { //dTstCCW<dTstCW
                    //CCW is shortest path
                    dDegToRotate = dTstCCW;
                    nRotationDirection = -1; //Counterclockwise
                }

            }
        } else {
            //((dCurrentPolar<0)&&(dCurrentPolar>=-180)) {
            //perhaps can combine with above conditional
            //but.. do this way to help debugging
            if ((dTargetPolar < 0) && (dTargetPolar >= -180)) {
                //    Current    Target    Correction           Dir
                //    --------   ------    -----------------
                //   -180 0    > -180 0    abx(target-current)  CCW
                //   -180 0    < -180 0    abx(target-current)  CW
                //At a negative current  polar angle
                if (dTargetPolar <= dCurrentPolar) { //target is also negative
                    //      C  |
                    //    T    |
                    // - - - - + - - - -
                    //         |
                    //    C    |
                    //      T  |
                    //if here need to move counterclockwise
                    dDegToRotate = dCurrentPolar - dTargetPolar; //make dDegToRotate pos
                    if (dDegToRotate == 0) nRotationDirection = 0;
                    else nRotationDirection = -1; //counterclockwise
                } else { //dTargetPolar>dCurrentPolar)
                    //      T  |
                    //    C    |
                    //         |
                    // - - - - + - - - -
                    //         |
                    //    T    |
                    //      C  |
                    //if here need to move clockwise
                    dDegToRotate = dTargetPolar - dCurrentPolar; //make dDegToRotate pos
                    nRotationDirection = 1; //Clockwise movement
                }
                //((dCurrentPolar<0)&&(dCurrentPolar>=-180)) {
            } else { //dTargetPolar >= 0 && dTargetPolar<=180
                //find shortest path
                //         | T
                //     C   |
                //         |
                // - - - - + - - - -
                //         |       T
                //         |
                //    Current    Target    Correction           Dir
                //    --------   ------    -----------------
                //   -180 0       0 180    abs(current)+target   CW
                //                0 180    current-(-180) + 180 - target  CCW
                //
                //   Ex Current = -8  Target =0  CW = abs(-8)+0= 8  CCW=(-8)+180 + 180 = 352

                dTstCW = Math.abs(dCurrentPolar) + dTargetPolar;
                dTstCCW = (dCurrentPolar + 180d) + (180d - dTargetPolar);
                if (dTstCCW >= dTstCW) {
                    //CW is shorter path
                    dDegToRotate = dTstCW;
                    if (dDegToRotate == 0) nRotationDirection = 0;
                    else nRotationDirection = 1; //Clockwise movement
                } else {
                    dDegToRotate = dTstCCW;
                    nRotationDirection = -1; //Counterclockwise
                }
            }
        }
        return dDegToRotate * (double) nRotationDirection;
    }

    /**
     * Ensure valid polar angle
     *
     * @param dPolarToFix The polar angle to check
     * @return The valid polar angle
     */
    private double fixPolar(double dPolarToFix) {

        double dRes = dPolarToFix;

        dPolarToFix %= 360.0d; //normalize  to 0 to 360;

        if (dPolarToFix > 180d) {
            //more than 180, so polar is actually negative
            dRes = dPolarToFix - 360d;
        } else if (dPolarToFix < -180d) {
            dRes = dPolarToFix + 360d;
        }
        return dRes;
    }
    /**
     * DIY setting of angle to hold
     * 
     * @param dPolarAngle
     */
    private void setDIYSetPoint(double dPolarAngle) {

        gdYawAngleToHold=dPolarAngle;
    }

    /**
     * Get the yaw angle from the NavX
     *
     * @return Yaw angle of the robot
     */
    public double getNavXYaw() {
        if (gnNavXStatus == NAVX_KAUAILABS_LIB) {
            if (navX.isConnected())
                return (double) (navX.getYaw());
            else return 0;
        } else if (gnNavXStatus == NAVX_QC_LIB) {
             
            try {
                AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
                Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gnNavXStatus = NAVX_QC_LIB;
                return (double) (-(angles.firstAngle)); //Darn FIRST implementation returns a neg angle
            } catch (Exception e) { //this does not catch condition of a failed navx.
                //Thread.currentThread().interrupt();
                gnNavXStatus = NAVX_ERROR;
                return 0;
            }
            

        } else {
            return (double) 0;
        }
    }

    public void setDebug(boolean bDebugToSet) {
        gbDebug=bDebugToSet;

    }

}


class MecanumV4AS {
    MecanumV4AS() {
    }

    public void operate(){
    }

    // public int getOdoHorizPos() {
    //     return 0;
    // }
    // public int getOdoVertPos() {
    //     return 0;
    // }
    public void stopMotors() {

    }

}
    
class SwerveV4AS {
    //General constants
    public int LF=0,RF=1,LB=2,RB=3 ;
    public final int CHASSIS_LEFT_FWD = 1; //Left wheels move forward when apply positive voltage
    //Includes NeverRest 20 orbitals
    public final int CHASSIS_RIGHT_FWD = 2; //Right wheels move forward when apply positive voltage
    //Includes NeverRest 3.7 orbitals
    private final double DEGTOPOS= 0.003d; //0=0deg 1=355deg 1/355 = .00282
    private final double[] ZERO_POS = new double[4];
    private final double[] NINETY_POS = new double[4];
    private final double[] NEG_NINETY_POS = new double[4];
    private final double[] ONEEIGHTY_POS = new double[4];
    private final double[] ZERO_VOLTS = new double[4];
    private final double[] ZERO_VOLTS_HIGH = new double[4];
    private final double[] ZERO_VOLTS_LOW = new double[4];
    private final double[] NINETY_VOLTS = new double[4];
    private final double[] NINETY_VOLTS_HIGH = new double[4];
    private final double[] NINETY_VOLTS_LOW = new double[4];
    private final double[] NEG_NINETY_VOLTS = new double[4];
    private final double[] NEG_NINETY_VOLTS_HIGH = new double[4];
    private final double[] NEG_NINETY_VOLTS_LOW = new double[4];
    private final double[] ONEEIGHTY_VOLTS = new double[4];
    private final double[] ONEEIGHTY_VOLTS_HIGH = new double[4];
    private final double[] ONEEIGHTY_VOLTS_LOW = new double[4];
    private final double POLAR_CHANGE_THRESH=33d;

    private final double NEG_NINETY_ADJ=.01;
    private final double NINETY_ADJ=-.01d;

    private final double VOLT_TOLERANCE_LOW = .8d;
    private final double VOLT_TOLERANCE_HIGH = 1.2d;

    //run constants
    private final int ROTATION_INPROGRESS = 1;
    private final int ROTATION_NODELAY = 0;
    private long glROTATION_TIME_ALLOW=500; // ms to allow module rotation to complete

    //Re-do:  the next 3 lines MUST match the object calling this one
    public final int CRAWL_FWD=0, CRAWL_BACK=2, CRAWL_LEFT=3, CRAWL_RIGHT=4, CRAWL_NONE=-1;
    public final int CCW = -1, CW = 1, NOSPIN = 0;
    public final int PREDEF_ROTATE_NONE=999;

    private String gstrClassName=this.getClass().getSimpleName();
    private int gnRotationStatus=ROTATION_NODELAY;
    private long glRotationStart=0;

    VectorPolarV4AS[] avecDir = new VectorPolarV4AS[4]; //setup reference to array of objects
    VectorPolarV4AS[] avecRot = new VectorPolarV4AS[4]; //setup reference to array of object
    VectorPolarV4AS[] avecRes = new VectorPolarV4AS[4]; //setup refrence to array of object

   
    //Motors
    private DcMotor[] amtrSM = new DcMotor[4];
    private DcMotor odoVert,odoHoriz;

    //Servos
    private Servo[] asrvoSM = new Servo[4];
    //analog inputs to check servo position
    private AnalogInput[] aanaSM = new AnalogInput[4];
    private double[] adServoVolts = new double[4];

    private boolean gbDebug=false;

  
    SwerveV4AS(OpMode opMode, int nReqLayout, String strMtrLFName, String strMtrLBName,
               String strMtrRFName, String strMtrRBName,
               DcMotor odoHoriz, DcMotor odoVert,
               String strSrvoLFName, String strSrvoLBName, String strSrvoRFName, String strSrvoRBName,
               long lRotationPause) {


        ZERO_POS[LF]=.50d; ZERO_VOLTS[LF]=1.65d; ZERO_VOLTS_LOW[LF]=ZERO_VOLTS[LF]*VOLT_TOLERANCE_LOW;ZERO_VOLTS_HIGH[LF]=ZERO_VOLTS[LF]*VOLT_TOLERANCE_HIGH;
        ZERO_POS[LB]=.50d; ZERO_VOLTS[LB]=1.65d; ZERO_VOLTS_LOW[LB]=ZERO_VOLTS[LB]*VOLT_TOLERANCE_LOW;ZERO_VOLTS_HIGH[LB]=ZERO_VOLTS[LB]*VOLT_TOLERANCE_HIGH;
        ZERO_POS[RF]=.50d; ZERO_VOLTS[RF]=1.66d; ZERO_VOLTS_LOW[RF]=ZERO_VOLTS[RF]*VOLT_TOLERANCE_LOW;ZERO_VOLTS_HIGH[RF]=ZERO_VOLTS[RF]*VOLT_TOLERANCE_HIGH;
        ZERO_POS[RB]=.50d; ZERO_VOLTS[RB]=1.65d; ZERO_VOLTS_LOW[RB]=ZERO_VOLTS[RB]*VOLT_TOLERANCE_LOW;ZERO_VOLTS_HIGH[RB]=ZERO_VOLTS[RB]*VOLT_TOLERANCE_HIGH;
        
        NINETY_POS[LF]=ZERO_POS[LF]-(90d*DEGTOPOS)+NINETY_ADJ+.0065; NINETY_VOLTS[LF]=2.45d; NINETY_VOLTS_LOW[LF]=NINETY_VOLTS[LF]*VOLT_TOLERANCE_LOW;NINETY_VOLTS_HIGH[LF]=NINETY_VOLTS[LF]*VOLT_TOLERANCE_HIGH;
        NINETY_POS[LB]=ZERO_POS[LB]-(90d*DEGTOPOS)+NINETY_ADJ+.0065-.035; NINETY_VOLTS[LB]=2.45d; NINETY_VOLTS_LOW[LB]=NINETY_VOLTS[LB]*VOLT_TOLERANCE_LOW;NINETY_VOLTS_HIGH[LB]=NINETY_VOLTS[LB]*VOLT_TOLERANCE_HIGH;
        NINETY_POS[RF]=ZERO_POS[RF]-(90d*DEGTOPOS)+NINETY_ADJ-.0065; NINETY_VOLTS[RF]=2.40d; NINETY_VOLTS_LOW[RF]=NINETY_VOLTS[RF]*VOLT_TOLERANCE_LOW;NINETY_VOLTS_HIGH[RF]=NINETY_VOLTS[RF]*VOLT_TOLERANCE_HIGH;
        NINETY_POS[RB]=ZERO_POS[RB]-(90d*DEGTOPOS)+NINETY_ADJ-.0065+.035; NINETY_VOLTS[RB]=2.40d; NINETY_VOLTS_LOW[RB]=NINETY_VOLTS[RB]*VOLT_TOLERANCE_LOW;NINETY_VOLTS_HIGH[RB]=NINETY_VOLTS[RB]*VOLT_TOLERANCE_HIGH;
        

        
        //LB: -.035
        //RB: +.035
        
        NEG_NINETY_POS[LF]=ZERO_POS[LF]+(90d*DEGTOPOS)+NEG_NINETY_ADJ+.00775-.0315; NEG_NINETY_VOLTS[LF]=.94d; NEG_NINETY_VOLTS_LOW[LF]=NEG_NINETY_VOLTS[LF]*VOLT_TOLERANCE_LOW;NEG_NINETY_VOLTS_HIGH[LF]=NEG_NINETY_VOLTS[LF]*VOLT_TOLERANCE_HIGH;
        NEG_NINETY_POS[LB]=ZERO_POS[LB]+(90d*DEGTOPOS)+NEG_NINETY_ADJ+.0077-.0315; NEG_NINETY_VOLTS[LB]=.94d; NEG_NINETY_VOLTS_LOW[LB]=NEG_NINETY_VOLTS[LB]*VOLT_TOLERANCE_LOW;NEG_NINETY_VOLTS_HIGH[LB]=NEG_NINETY_VOLTS[LB]*VOLT_TOLERANCE_HIGH;
        NEG_NINETY_POS[RF]=ZERO_POS[RF]+(90d*DEGTOPOS)+NEG_NINETY_ADJ-.0077; NEG_NINETY_VOLTS[RF]=.89d; NEG_NINETY_VOLTS_LOW[RF]=NEG_NINETY_VOLTS[RF]*VOLT_TOLERANCE_LOW;NEG_NINETY_VOLTS_HIGH[RF]=NEG_NINETY_VOLTS[RF]*VOLT_TOLERANCE_HIGH;
        NEG_NINETY_POS[RB]=ZERO_POS[RB]+(90d*DEGTOPOS)+NEG_NINETY_ADJ-.0077; NEG_NINETY_VOLTS[RB]=.89d; NEG_NINETY_VOLTS_LOW[RB]=NEG_NINETY_VOLTS[RB]*VOLT_TOLERANCE_LOW;NEG_NINETY_VOLTS_HIGH[RB]=NEG_NINETY_VOLTS[RB]*VOLT_TOLERANCE_HIGH;
        
        //-.035
        //-.0325 - 2inch forward
        //-.0315 - .5-1in f
        
        
        ONEEIGHTY_POS[LF]=0;ONEEIGHTY_VOLTS[LF]=3.11d;ONEEIGHTY_VOLTS_LOW[LF]=ONEEIGHTY_VOLTS[LF]*VOLT_TOLERANCE_LOW;ONEEIGHTY_VOLTS_HIGH[LF]=ONEEIGHTY_VOLTS[LF]*VOLT_TOLERANCE_HIGH;
        ONEEIGHTY_POS[LB]=0;ONEEIGHTY_VOLTS[LB]=3.11d;ONEEIGHTY_VOLTS_LOW[LB]=ONEEIGHTY_VOLTS[LF]*VOLT_TOLERANCE_LOW;ONEEIGHTY_VOLTS_HIGH[LB]=ONEEIGHTY_VOLTS[LB]*VOLT_TOLERANCE_HIGH;
        ONEEIGHTY_POS[RF]=0;ONEEIGHTY_VOLTS[RF]=3.13d;ONEEIGHTY_VOLTS_LOW[RF]=ONEEIGHTY_VOLTS[RF]*VOLT_TOLERANCE_LOW;ONEEIGHTY_VOLTS_HIGH[RF]=ONEEIGHTY_VOLTS[RF]*VOLT_TOLERANCE_HIGH;
        ONEEIGHTY_POS[RB]=0;ONEEIGHTY_VOLTS[RB]=3.12d;ONEEIGHTY_VOLTS_LOW[RB]=ONEEIGHTY_VOLTS[RB]*VOLT_TOLERANCE_LOW;ONEEIGHTY_VOLTS_HIGH[RB]=ONEEIGHTY_VOLTS[LF]*VOLT_TOLERANCE_HIGH;
       

        glROTATION_TIME_ALLOW=lRotationPause;
        int nIdx;


        for(nIdx=0;nIdx<4;nIdx++) {
            avecDir[nIdx]=new VectorPolarV4AS(); //crate objects ans references
            avecRot[nIdx]=new VectorPolarV4AS(); //crete objects and references
            avecRes[nIdx]=new VectorPolarV4AS(); //crete objects and references

        }

        initMotors(opMode, nReqLayout, strMtrLFName, strMtrLBName, strMtrRFName, strMtrRBName);
                //strMtrOdoHorizName, strMtrOdoVertName);
        initServos(opMode,"srvoLeftFront","srvoLeftBack","srvoRightFront","srvoRightBack",
                "anaRotationLF","anaRotationLB","anaRotationRF","anaRotationRB");

        //odometry modules are using the encoder ports of other DcMotors outside of this class
        //The motors must be initialized outside of this class, and passed as a DCMotor type
        this.odoHoriz=odoHoriz;
        this.odoVert=odoVert;
        resetOdo();

    }

    /**
     * 
     * @param OpMode        OpMode to use
     * @param dDirPwr       linear directional power 
     * @param dDirPolar     linear direction polar
     * @param nRobotSpin    rotation of robot
     * @param dRotPwr       rotational power
     */
    public void operate(OpMode opMode, double dDirPolar,double dDirPwr,int nRobotSpin, double dRotPwr ) {

//opMode.telemetry.addData(gstrClassName,"dirpolar:%.2f dirpwr:%.2f spin:%d rotpwr:%.2f ",
//    dDirPolar, dDirPwr,nRobotSpin,dRotPwr);
//opMode.telemetry.update();
            //check if this is a new state
        setvectorSwerveModulesDir(opMode, dDirPwr, dDirPolar);
        setvectorSwerveModulesRot(nRobotSpin, dRotPwr);
        setvectorSwerveModulesRes(opMode);
        runSwerveModules(opMode);

    } //end operate

    /**
     * Crawl the robot
     * @param opMode op mode
     * @param nCrawling crawling direcion (CRAWL_LEFT, CRAWL_RIGHT...
     * @param dDirPwr power to apply
     */
    public void crawlRobot(OpMode opMode,int nCrawling,double dDirPwr) {
        boolean bSuccess;
        if(nCrawling==CRAWL_RIGHT) {
            bSuccess= rotateServoToPolar90Op(opMode);
            if(bSuccess) {
                pwrMotors(dDirPwr);
            }
        } if(nCrawling==CRAWL_LEFT) {
            bSuccess= rotateServoToNegPolar90Op(opMode);
            if(bSuccess) {
                pwrMotors(dDirPwr);
            }
        } if(nCrawling==CRAWL_FWD) {
            bSuccess= rotateServoToPolar0Op(opMode);
            if(bSuccess) {
                pwrMotors(dDirPwr);
            }
        } if(nCrawling==CRAWL_BACK) {
            bSuccess= rotateServoToPolar0Op(opMode);
            if (bSuccess) {
                pwrMotors(-dDirPwr);
            }
        }
    } //end crawlRobot



    /**
    *  Set the cartesian coord for each swerve module's linear direction
     * @param dDirPwr Directional power/
     * @param dDirPolar Directional magnitude 
     * @return The largest polar angle difference between now and before
     */ 
    private void setvectorSwerveModulesDir(OpMode opMode, double dDirPwr, double dDirPolar) {

        int nIdx;
        for(nIdx=0;nIdx<=3;nIdx++) {
            avecDir[nIdx].setVectorUsingPolar(dDirPwr,dDirPolar);
//opMode.telemetry.addData(gstrClassName, "set:%.2f Mag:%.2f x:%.2f y:%.2f",avecDir[nIdx].getPolarAngle(),avecDir[nIdx].getMagnitude(),
//avecDir[nIdx].getX(),avecDir[nIdx].getY());

         
        }
//opMode.telemetry.update();
     
    }

    /**
     * Set the cartesian coord for each serve modules's rotation
     * @param nRobotSpin
     * @param dRotPwr
     */ 
    private void setvectorSwerveModulesRot(int nRobotSpin, double dRotPwr){
        //Optimal spin angles
        //   
        //    LF   /      \   RF 
        //
        //    LB   \      /   RB 
        //below is optimal, but slow
        
        if(nRobotSpin==CW) { //CW spin
            avecRot[LF].setVectorUsingPolar(dRotPwr,45d);
            avecRot[LB].setVectorUsingPolar(dRotPwr,-45d);
            avecRot[RF].setVectorUsingPolar(-dRotPwr,-45d);
            avecRot[RB].setVectorUsingPolar(-dRotPwr,45d);
        } else if(nRobotSpin==CCW) {//CCW
            avecRot[LF].setVectorUsingPolar(-dRotPwr,45d);
            avecRot[LB].setVectorUsingPolar(-dRotPwr,-45d);
            avecRot[RF].setVectorUsingPolar(dRotPwr,-45d);
            avecRot[RB].setVectorUsingPolar(dRotPwr,45d);
        } else { //NOSPIN
            avecRot[LF].setVectorUsingPolar(0,0);
            avecRot[LB].setVectorUsingPolar(0,0);
            avecRot[RF].setVectorUsingPolar(0,0);
            avecRot[RB].setVectorUsingPolar(0,0);
        }

if(gbDebug) System.out.printf("   RB Rot:%.2f mag:%.2f\n",avecRot[RB].getPolarAngle(),avecRot[RB].getMagnitude());
    }

    /**
     *  run the swerve modules
     * @param  dLargestPolarChange Largest polar angle change
     */
    private  void runSwerveModules(OpMode opMode) {
        int nIdx;
        double dHighestMagnitude=0d,dCheck,dScale=1d;
        double dLargestPolarChange=0d, dDiff=0d, dMax=0d;
        double dPos;
        boolean bMotion=false;
        boolean bAllOkay=true;
        boolean[] abChk= new boolean[4];
        double[] adVoltage= new double[4];
        double[] adPos= new double[4];

        //first, see if any there was any motion requested
        bMotion=false;
        for (nIdx = 0; nIdx < 4; nIdx++) {
            //recall the dir and rot magnitude comes from user/correctional inputs
            //the crucial point is that mag and polar is not calculated  for dir and rot: values come from UsingPolar
            //only the res magnitude and polar is calculated, and not from inputs.  res value comes from UsingCartesian
            if(avecDir[nIdx].getMagnitude()!=0) bMotion=true;
            if(avecRot[nIdx].getMagnitude()!=0) bMotion=true;
        }
        //set angles and magnitudes to use
        for (nIdx = 0; nIdx < 4; nIdx++) {
//opMode.telemetry.addData(gstrClassName, "run b4:%.2f Mag:%.2f x:%d",avecRes[nIdx].getPolarAngle(),avecRes[nIdx].getMagnitude());           
//opMode.telemetry.addData(gstrClassName, "    b4:%.2f Mag:%.2f",avecDir[nIdx].getPolarAngle(),avecDir[nIdx].getMagnitude());           
            if(((avecRes[nIdx].getPolarPrev()>50)&&(avecRes[nIdx].getPolarPrev()<170)&& /*50*/
                (avecRes[nIdx].getPolarAngle()>50)&&(avecRes[nIdx].getPolarAngle()<170))||
                ((avecRes[nIdx].getPolarPrev()<-50)&&(avecRes[nIdx].getPolarPrev()>-170)&&
                (avecRes[nIdx].getPolarAngle()<-50)&&(avecRes[nIdx].getPolarAngle()>-170))) {
                //if got here, have to continuously rotate
                if(bMotion) { //if motion requested
                    //avecDir[nIdx].setMagnitudeToUse(avecDir[nIdx].getMagnitude());
                    //avecDir[nIdx].setPolarAngleToUse(avecDir[nIdx].getPolarAngle());
                    //avecRes[nIdx].setMagnitudeToUse(avecRes[nIdx].getMagnitude());
                    //avecRes[nIdx].setPolarAngleToUse(avecRes[nIdx].getPolarAngle());
                    if((avecRes[nIdx].getMagnitude()>0)&&(avecRes[nIdx].getMagnitudePrev()<0)){  //sign flipped
                        //dont want to reverse wheels
                        //if got here, previous was negative
                        //magnitude is always positive, its optimal can be negative
                        //take the possible  negative
                        avecDir[nIdx].setMagnitudeToUse(avecDir[nIdx].getMagnitudeOptimal());
                        avecDir[nIdx].setPolarAngleToUse(avecDir[nIdx].getPolarAngleOptimal());
                        avecRes[nIdx].setMagnitudeToUse(avecRes[nIdx].getMagnitudeOptimal());
                        avecRes[nIdx].setPolarAngleToUse(avecRes[nIdx].getPolarAngleOptimal());
                    } else if((avecRes[nIdx].getMagnitude()<0)&&(avecRes[nIdx].getMagnitudePrev()>0)) {  //sign flipped
                        //dont want to reverse wheels,
                        //if got here, previous was positive
                        //use the polar
                        avecDir[nIdx].setMagnitudeToUse(avecDir[nIdx].getMagnitude());
                        avecDir[nIdx].setPolarAngleToUse(avecDir[nIdx].getPolarAngle());
                        avecRes[nIdx].setMagnitudeToUse(avecRes[nIdx].getMagnitude());
                        avecRes[nIdx].setPolarAngleToUse(avecRes[nIdx].getPolarAngle());
                    } else if((avecRes[nIdx].getMagnitude()<0)&&(avecRes[nIdx].getMagnitudePrev()<0)) {  //both same negative
                        avecDir[nIdx].setMagnitudeToUse(avecDir[nIdx].getMagnitudeOptimal());
                        avecDir[nIdx].setPolarAngleToUse(avecDir[nIdx].getPolarAngleOptimal());
                        avecRes[nIdx].setMagnitudeToUse(avecRes[nIdx].getMagnitudeOptimal());
                        avecRes[nIdx].setPolarAngleToUse(avecRes[nIdx].getPolarAngleOptimal());
                    } else { //both same positive or zero
                        avecDir[nIdx].setMagnitudeToUse(avecDir[nIdx].getMagnitude());
                        avecDir[nIdx].setPolarAngleToUse(avecDir[nIdx].getPolarAngle());
                        avecRes[nIdx].setMagnitudeToUse(avecRes[nIdx].getMagnitude());
                        avecRes[nIdx].setPolarAngleToUse(avecRes[nIdx].getPolarAngle());
                    }

                } else { //no motion requested (no direction stick move or button press)
                    avecDir[nIdx].setMagnitudeToUse(0d);
                    avecDir[nIdx].setPolarAngleToUse(avecDir[nIdx].getPolarPrev());

                }

            }
            else { //can jump rotate
                if(bMotion){ //if motion requested
                    avecDir[nIdx].setMagnitudeToUse(avecDir[nIdx].getMagnitudeOptimal());
                    avecDir[nIdx].setPolarAngleToUse(avecDir[nIdx].getPolarAngleOptimal());
                    avecRes[nIdx].setMagnitudeToUse(avecRes[nIdx].getMagnitudeOptimal());
                    avecRes[nIdx].setPolarAngleToUse(avecRes[nIdx].getPolarAngleOptimal());
                } else { //no motion requested
                    avecDir[nIdx].setMagnitudeToUse(0d);
                    avecDir[nIdx].setPolarAngleToUse(avecDir[nIdx].getPolarPrev());
                    avecRes[nIdx].setMagnitudeToUse(0d);
                    avecRes[nIdx].setPolarAngleToUse(avecRes[nIdx].getPolarPrev());
                }
            }
//opMode.telemetry.addData(gstrClassName, "run af:%.2f Mag:%.2f",avecRes[nIdx].getPolarAngleToUse(),avecRes[nIdx].getMagnitudeToUse());
//opMode.telemetry.update();
        }

        //set servos first
        for (nIdx = 0; nIdx < 4; nIdx++) {
            //dPos = ZERO_POS[nIdx] - (avecRes[nIdx].getPolarAngleToUse() * DEGTOPOS);
            dPos=calcPosFromPolar(avecRes[nIdx].getPolarAngleToUse(),nIdx);
            asrvoSM[nIdx].setPosition(dPos);
        }

        //next, find the largest polar change
        for(nIdx=0;nIdx<=3;nIdx++) {
            dDiff=Math.abs(avecDir[nIdx].getPolarPrev()-avecDir[nIdx].getPolarAngleToUse());
            if(dDiff>dMax) dMax=dDiff;
        }
        dLargestPolarChange=dMax;
        if(dLargestPolarChange>POLAR_CHANGE_THRESH) {
            gnRotationStatus=ROTATION_INPROGRESS;
            glRotationStart=System.currentTimeMillis();
            //need to delay
//opMode.telemetry.addData(gstrClassName,"delaying, polarChange:%.2f",dLargestPolarChange);

            for(nIdx=0;nIdx<4;nIdx++) {
                amtrSM[nIdx].setPower(0);//no motor power while moving
            }
        }

        if(gnRotationStatus==ROTATION_INPROGRESS) {
            bAllOkay=true;
            for(nIdx=0;nIdx<4;nIdx++) {
                if((abChk[nIdx]=chkServoPosFromPolar(asrvoSM[nIdx].getPosition(),nIdx))==false)
                    bAllOkay=false;
                adPos[nIdx]=asrvoSM[nIdx].getPosition();
                adVoltage[nIdx]=aanaSM[nIdx].getVoltage();
                opMode.telemetry.addData(gstrClassName,"%s %d targetPosition:%.2f volt:%.2f",
                        abChk[nIdx],nIdx,asrvoSM[nIdx].getPosition(),aanaSM[nIdx].getVoltage());
            }
            /* only do the following in emergency debug...
            while(gnRotationStatus==ROTATION_INPROGRESS) {
                for(nIdx=0;nIdx<4;nIdx++) {
                    opMode.telemetry.addData(gstrClassName, "%s %d targetPosition:%.2f volt:%.2f",
                            abChk[nIdx], nIdx, asrvoSM[nIdx].getPosition(), aanaSM[nIdx].getVoltage());
                }
                opMode.telemetry.update();
            }
            */
            if(bAllOkay) {
                gnRotationStatus=ROTATION_NODELAY;
                glRotationStart=0;
            } else {
                if((System.currentTimeMillis()-glRotationStart)>glROTATION_TIME_ALLOW) {
                    //waited long enough
                    gnRotationStatus=ROTATION_NODELAY;
                    glRotationStart=0;
                } else {
                    //still need to wait
                    return;
                }
            }
        }

        //if got here, can power wheels
        //find the highest resultant magnitude so can scale
        dHighestMagnitude=0;
        for(nIdx=0;nIdx<4;nIdx++) {
            dCheck=Math.abs(avecRes[nIdx].getMagnitudeToUse());
            if(dCheck>dHighestMagnitude) {
                dHighestMagnitude=dCheck;
            }
        }
        if(dHighestMagnitude>1d) dScale=1d/dHighestMagnitude;
        else dScale=1d;
//opMode.telemetry.addData(gstrClassName,"dscale is:%.2f",dScale);
//opMode.telemetry.update();

        for(nIdx=0;nIdx<4;nIdx++) {
            amtrSM[nIdx].setPower(avecRes[nIdx].getMagnitudeToUse()*dScale);
        }
    }

    /**
     *
     * @param opMode  OpMode to use
     * Sets the resultant vector from direction vector and rotation vector
     */
    private void setvectorSwerveModulesRes(OpMode opMode) {
        int nIdx;
        
//opMode.telemetry.addData(gstrClassName, "setting swerve");

        for(nIdx=0;nIdx<=3;nIdx++) {
//opMode.telemetry.addData(gstrClassName, "run b4:%.2f Mag:%.2f",avecRes[nIdx].getPolarAngle(),
//avecRes[nIdx].getMagnitude());           
//opMode.telemetry.addData(gstrClassName, "    dir:%.2f Mag:%.2f x:%.2f y:%.2f",avecDir[nIdx].getPolarAngle(),
//avecDir[nIdx].getMagnitude(),avecDir[nIdx].getX(),avecDir[nIdx].getY());           
//opMode.telemetry.addData(gstrClassName, "    rot:%.2f Mag:%.2f x:%.2f y:%.2f",avecRot[nIdx].getPolarAngle(),
//avecRot[nIdx].getMagnitude(),avecRot[nIdx].getX(),avecRot[nIdx].getY());           

            //set the resultant vector
            avecRes[nIdx].setVectorUsingCartesian( //no magnitude.. already built into x,y
                (avecDir[nIdx].getX()+ avecRot[nIdx].getX()),
                (avecDir[nIdx].getY()+avecRot[nIdx].getY()));
            
//opMode.telemetry.addData(gstrClassName, "run af:%.2f Mag:%.2f x:%.2f y:%.2f",avecRes[nIdx].getPolarAngle(),avecRes[nIdx].getMagnitude(),
//avecRes[nIdx].getMagnitude(),avecRes[nIdx].getX(),avecRes[nIdx].getY());           
//opMode.telemetry.addData(gstrClassName, "    dir:%.2f Mag:%.2f x:%.2f y:%.2f",avecDir[nIdx].getPolarAngle(),
//avecDir[nIdx].getMagnitude(),avecDir[nIdx].getX(),avecDir[nIdx].getY());           
//opMode.telemetry.addData(gstrClassName, "    rot:%.2f Mag:%.2f x:%.2f y:%.2f",avecRot[nIdx].getPolarAngle(),
//avecRot[nIdx].getMagnitude(),avecRot[nIdx].getX(),avecRot[nIdx].getY());           

        }
//opMode.telemetry.update();
       
        
if(gbDebug) System.out.printf("LF Res:%.2f mag:%.2f x:%.2f y:%.2f =>",avecRes[LF].getPolarAngle(),avecRes[LF].getMagnitude(),avecRes[LF].getX(),avecRes[LF].getY());
if(gbDebug) System.out.printf("Res:%.2f mag:%.2f x:%.2f y:%.2f\n",avecRes[LF].getPolarAngleOptimal(),avecRes[LF].getMagnitudeOptimal(),avecRes[LF].getX(),avecRes[LF].getY());
if(gbDebug) System.out.printf("   Dir:%.2f mag:%.2f x:%.2f y:%.2f\n",avecDir[LF].getPolarAngle(),avecDir[LF].getMagnitude(),avecDir[LF].getX(),avecDir[LF].getY());
if(gbDebug) System.out.printf("   Rot:%.2f mag:%.2f x:%.2f y:%.2f\n",avecRot[LF].getPolarAngle(),avecRot[LF].getMagnitude(),avecRot[LF].getX(),avecRot[LF].getY());
if(gbDebug) System.out.printf("LB Res:%.2f mag:%.2f x:%.2f y:%.2f=>",avecRes[LB].getPolarAngle(),avecRes[LB].getMagnitude(),avecRes[LB].getX(),avecRes[LB].getY());
if(gbDebug) System.out.printf("Res:%.2f mag:%.2f x:%.2f y:%.2f\n",avecRes[LB].getPolarAngleOptimal(),avecRes[LB].getMagnitudeOptimal(),avecRes[LB].getX(),avecRes[LB].getY());
if(gbDebug) System.out.printf("   Dir:%.2f mag:%.2f x:%.2f y:%.2f\n",avecDir[LB].getPolarAngle(),avecDir[LB].getMagnitude(),avecDir[LB].getX(),avecDir[LB].getY());
if(gbDebug) System.out.printf("   Rot:%.2f mag:%.2f x:%.2f y:%.2f\n",avecRot[LB].getPolarAngle(),avecRot[LB].getMagnitude(),avecRot[LB].getX(),avecRot[LB].getY());
if(gbDebug) System.out.printf("RF Res:%.2f mag:%.2f x:%.2f y:%.2f=>",avecRes[RF].getPolarAngle(),avecRes[RF].getMagnitude(),avecRes[RF].getX(),avecRes[RF].getY());
if(gbDebug) System.out.printf("Res:%.2f mag:%.2f x:%.2f y:%.2f\n",avecRes[RF].getPolarAngleOptimal(),avecRes[RF].getMagnitudeOptimal(),avecRes[RF].getX(),avecRes[RF].getY());
if(gbDebug) System.out.printf("   Dir:%.2f mag:%.2f x:%.2f y:%.2f\n",avecDir[RF].getPolarAngle(),avecDir[RF].getMagnitude(),avecDir[RF].getX(),avecDir[RF].getY());
if(gbDebug) System.out.printf("   Rot:%.2f mag:%.2f x:%.2f y:%.2f\n",avecRot[RF].getPolarAngle(),avecRot[RF].getMagnitude(),avecRot[RF].getX(),avecRot[RF].getY());
if(gbDebug) System.out.printf("RB Res:%.2f mag:%.2f x:%.2f y:%.2f=>",avecRes[RB].getPolarAngle(),avecRes[RB].getMagnitude(),avecRes[RB].getX(),avecRes[RB].getY());
if(gbDebug) System.out.printf("Res:%.2f mag:%.2f x:%.2f y:%.2f\n",avecRes[RB].getPolarAngleOptimal(),avecRes[RB].getMagnitudeOptimal(),avecRes[RB].getX(),avecRes[RB].getY());
if(gbDebug) System.out.printf("   Dir:%.2f mag:%.2f x:%.2f y:%.2f\n",avecDir[RB].getPolarAngle(),avecDir[RB].getMagnitude(),avecDir[RB].getX(),avecDir[RB].getY());
if(gbDebug) System.out.printf("   Rot:%.2f mag:%.2f x:%.2f y:%.2f\n",avecRot[RB].getPolarAngle(),avecRot[RB].getMagnitude(),avecRot[RB].getX(),avecRot[RB].getY());

    }

    /**
     * Stop all motors
     */
    void stopMotors() {
        int nIdx;
        for(nIdx=0;nIdx<4;nIdx++)
            amtrSM[nIdx].setPower(0d);
        return;
    }

    /**
     * power all motors in same direction
     * @param dPwr power to apply
     */
    void pwrMotors(double dPwr) {
        int nIdx;
        for(nIdx=0;nIdx<4;nIdx++)
            amtrSM[nIdx].setPower(dPwr);
        return;
    }
    void pwrMotorsAll (double dPwrLF, double dPwrLB, double dPwrRF, double dPwrRB){
        amtrSM[LF].setPower(dPwrLF);
        amtrSM[LB].setPower(dPwrLB);
        amtrSM[RF].setPower(dPwrRF);
        amtrSM[RB].setPower(dPwrRB);

    }


    /**
     * Set all motors a distance for run_to_position
      * @param nDist
     */
    public void setMotorRunToTargets(int nDist) {
        int nIdx;
        for(nIdx=0;nIdx<4;nIdx++) {
            amtrSM[nIdx].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            amtrSM[nIdx].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            amtrSM[nIdx].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            amtrSM[nIdx].setTargetPosition(nDist);
        }
        return;
    }

    /**
     * Set all motors a spinning distance
     *
     * @param linopMode
     * @param nDist     to spin
     */
    public void setMotorSpinToTargets(int nDist) {
        amtrSM[LF].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        amtrSM[LF].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        amtrSM[LF].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        amtrSM[LF].setTargetPosition(nDist);

        amtrSM[RF].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        amtrSM[RF].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        amtrSM[RF].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        amtrSM[RF].setTargetPosition(-nDist);

        amtrSM[LB].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        amtrSM[LB].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        amtrSM[LB].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        amtrSM[LB].setTargetPosition(nDist);

        amtrSM[RB].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        amtrSM[RB].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        amtrSM[RB].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        amtrSM[RB].setTargetPosition(-nDist);

        return;
    }

    /**
     * Check to see if all motors are busy
     * @return true if any motor is busy false when all motors are not busy
     */
    boolean chkMotorsBusy() {
        if(amtrSM[LF].isBusy()||
               (amtrSM[LB].isBusy()) ||
               (amtrSM[RF].isBusy()) ||
               (amtrSM[RB].isBusy()) ){
            return true;
        }
        return false;

    } /**
     * Check if all motors reached position oncne
     * @param linOpMode
     * @param nTolerance encoder count range (MUST BE POS)
     * @param lTimeout max time to wait
     * @return
     */
    boolean chkMotorsReachedPosLOp(LinearOpMode linOpMode, int nTolerance , long lTimeout) {

        boolean[] abReached = new boolean[4];
        int[] anHigh = new int[4];
        int[] anLow = new int[4];
        long lTimestamp=System.currentTimeMillis();
        int nTarget,nTol;
        int nIdx;

        nTol=Math.abs(nTolerance);

        //initialize
        for(nIdx=0;nIdx<4;nIdx++) {
            abReached[nIdx]=false;
            if((nTarget=amtrSM[nIdx].getTargetPosition())>=0) {
                anHigh[nIdx] = nTarget + nTol;
                anLow[nIdx] = nTarget-nTol;
            } else {
                anHigh[nIdx] = nTarget + nTol;
                anLow[nIdx] = nTarget-nTol;
            }

        }
        //check until timeout
        while ((linOpMode.opModeIsActive())&&((System.currentTimeMillis()-lTimestamp)<lTimeout))  {
            for(nIdx=0;nIdx<4;nIdx++) {
                if((amtrSM[nIdx].getCurrentPosition()>=anLow[nIdx])&&
                    (amtrSM[nIdx].getCurrentPosition()<=anHigh[nIdx])) {
                    abReached[nIdx]=true;
                }
            }
            linOpMode.telemetry.addData(gstrClassName,"%d diff LF:%d RF:%d LB:%d RB:%d",
                    nTolerance,
                    amtrSM[LF].getTargetPosition()-amtrSM[LF].getCurrentPosition(),
                    amtrSM[RF].getTargetPosition()-amtrSM[RF].getCurrentPosition(),
                    amtrSM[LB].getTargetPosition()-amtrSM[LB].getCurrentPosition(),
                    amtrSM[RF].getTargetPosition()-amtrSM[RB].getCurrentPosition());
            //linOpMode.telemetry.update();
            if(abReached[LF]&&abReached[LB]&&abReached[RF]&&abReached[RB]) return true;
        }

        return false;
    }
    void displayMotorInfo(OpMode opMode) {

        opMode.telemetry.addData(gstrClassName,"LF:%d(%s) RF:%d(%s) LB:%d(%s) RB:%d(%s)",
            amtrSM[LF].getTargetPosition()-amtrSM[LF].getCurrentPosition(),
                amtrSM[LF].isBusy(),
                amtrSM[RF].getTargetPosition()-amtrSM[RF].getCurrentPosition(),
                amtrSM[RF].isBusy(),
                amtrSM[LB].getTargetPosition()-amtrSM[LB].getCurrentPosition(),
                amtrSM[LB].isBusy(),
                amtrSM[RB].getTargetPosition()-amtrSM[RB].getCurrentPosition(),
                 amtrSM[RB].isBusy());
    }


    /**
     * Get the vertical position
     * @return the encoder count of the vertical odometry pod
     */
    // public int getOdoVertPos() {
    //     return odoVert.getCurrentPosition();
    // }
    // /**
    //  * Get the horizontal position
    //  * @return the encoder count of the horizontal odometry pod
    //  */
    // public int getOdoHorizPos() {
    //     return odoHoriz.getCurrentPosition();
    // }

    public int getOdoVertPos() {
        if(odoVert==null)
           return amtrSM[RF].getCurrentPosition();
        else return odoVert.getCurrentPosition();
    }
    // /**
    //  * Get the horizontal position
    //  * @return the encoder count of the horizontal odometry pod
    //  */
    public int getOdoHorizPos() {
        if(odoVert==null)
            return amtrSM[RF].getCurrentPosition();
        else return odoHoriz.getCurrentPosition();
    }
    public double getDirX(int nID) {
        if((nID<0)||(nID>3)) return 0;
        return(avecDir[nID].getX());   
    }
    public double getDirY(int nID) {
        if((nID<0)||(nID>3)) return 0;
        return(avecDir[nID].getY());   
    }
    public double getDirPolar(int nID) {
        if((nID<0)||(nID>3)) return 0;
        return(avecDir[nID].getPolarAngle());   
    }
    public double getRotPolar(int nID) {
        if((nID<0)||(nID>3)) return 0;
        return(avecRot[nID].getPolarAngle());   
    }
    public double getResPolar(int nID) {
        if((nID<0)||(nID>3)) return 0;
        return(avecRes[nID].getPolarAngle());   
    }
    public double getDirMagnitude(int nID) {
        if((nID<0)||(nID>3)) return 0;
        return(avecDir[nID].getMagnitude());  
    }
    public double getRotMagnitude(int nID) {
        if((nID<0)||(nID>3)) return 0;
        return(avecRot[nID].getMagnitude());  
    }
    public double getResMagnitude(int nID) {
        if((nID<0)||(nID>3)) return 0;
        return(avecRes[nID].getMagnitude());  
    }

    public void setDebug(boolean bDebugToSet) {
        gbDebug=bDebugToSet;
    }
    
/**
     * init motors based on nReqLayout.
     * motors must be named mtrLeftFront, mtrLeftBack, mtrRightFront, mtrRightBack
     *
     * @param opMode       used to find chassis motors in robot configuration
     * @param nReqLayout   Layout requested by user; Either CHASSIS_LEFT_FWD or CHASSIS_RIGHT_FWD
     *                     CHASSIS_LEFT_FWD use this when applying positive pwr to left motors drives the left wheels forward
     *                     This setting implies right wheels move forward with negative pwr
     *                     This setting is the default if an invalid nReqLayout sent
     *                     Use this for: NeverRest 20 Orbital motors
     *                     CHASSIS_RIGHT_FWD use this when applying positive pwr to right motors drives the right wheels forward
     *                     Setting implies left wheels move forward with negative pwr
     *                     Use this for: NeverRest 3.7 Orbital motors
     * @param strMtrLFName name used in robot configuration for the left front motor
     * @param strMtrLBName name used in robot configuration for the left back motor
     * @param strMtrRFName name used in robot configuration for the right front motor
     * @param strMtrRBName name used in robot configuration for the right back motor
     * //@param strOdoHorizName name used in robot configuration for the horizontal odometry pod
     * //@param strOdoVertName name used in robot configuration for the vertical odometry pod
     *                     return: none. Sets global variable gnChassisLayout based on parameter nReqLayout
     */
    private void initMotors(OpMode opMode, int nReqLayout, String strMtrLFName, String strMtrLBName,
                            String strMtrRFName, String strMtrRBName) {
                            //String strOdoHorizName, String strOdoVertName) {
        //setup motors
        // Define and Initialize Motors
        //no Try..Catch here... Need to have everything stop if motors cannot be setup
        amtrSM[LB] = opMode.hardwareMap.get(DcMotor.class, strMtrLBName);
        amtrSM[RB] = opMode.hardwareMap.get(DcMotor.class, strMtrRBName);
        amtrSM[LF] = opMode.hardwareMap.get(DcMotor.class, strMtrLFName);
        amtrSM[RF] = opMode.hardwareMap.get(DcMotor.class, strMtrRFName);
        if (nReqLayout == CHASSIS_RIGHT_FWD) {
            amtrSM[LF].setDirection(DcMotor.Direction.FORWARD); // Positive input rotates clockwise
            amtrSM[RF].setDirection(DcMotor.Direction.REVERSE);// Positive input rotates counter clockwise
            amtrSM[LB].setDirection(DcMotor.Direction.FORWARD); // Positive input rotates clockwise
            amtrSM[RB].setDirection(DcMotor.Direction.REVERSE);// Positive input rotates counter clockwise
            //gnChassisLayout = nReqLayout;
        } else if (nReqLayout == CHASSIS_LEFT_FWD) {
            //set motors to NeverRest 3.7 Orbital
            amtrSM[LF].setDirection(DcMotor.Direction.REVERSE); // Positive input rotates counter clockwise
            amtrSM[RF].setDirection(DcMotor.Direction.FORWARD);// Positive input rotates clockwise
            amtrSM[LB].setDirection(DcMotor.Direction.REVERSE); // Positive input rotates counter clockwise
            amtrSM[RB].setDirection(DcMotor.Direction.FORWARD);// Positive input rotates clockwise
            //gnChassisLayout = nReqLayout;
        } else { //unsupported requested layout sent... use CHASSIS_LEFT_FWD settings;
            amtrSM[LF].setDirection(DcMotor.Direction.FORWARD); // Positive input rotates clockwise
            amtrSM[RF].setDirection(DcMotor.Direction.REVERSE);// Positive input rotates counter clockwise
            amtrSM[LB].setDirection(DcMotor.Direction.FORWARD); // Positive input rotates clockwise
            amtrSM[RB].setDirection(DcMotor.Direction.REVERSE);// Positive input rotates counter clockwise
            //gnChassisLayout = CHASSIS_LEFT_FWD;
        }
        // Stop all chassis motion by setting each axis value to zero
        amtrSM[LF].setPower(0d);
        amtrSM[RF].setPower(0d);
        amtrSM[LB].setPower(0d);
        amtrSM[RB].setPower(0d);
    }

    /**
     * init servos
     * servos must be named mtrLeftFront, mtrLeftBack, mtrRightFront, mtrRightBack
     *
     * @param opMode       used to find chassis motors in robot configuration
     * @param strSrvoLFName name used in robot configuration for the left front motor
     * @param strSrvoLBName name used in robot configuration for the left back motor
     * @param strSrvoRFName name used in robot configuration for the right front motor
     * @param strSrvoRBName name used in robot configuration for the right back motor
     *                     return: none. Sets global variable gnChassisLayout based on parameter nReqLayout
     */


    /**
     * init Odometery pods
     *
     */
    private void resetOdo() {
        if(odoHoriz!=null) odoHoriz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(odoVert!=null) odoVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    private void initServos(OpMode opMode,
                            String strSrvoLFName, String strSrvoLBName,
                            String strSrvoRFName, String strSrvoRBName,
                            String strAnaLFName, String strAnaLBName,
                            String strAnaRFName, String strAnaRBName) {
        //setup servos
        // Define and init servos
        //no Try..Catch here... Need to have everything stop if motors cannot be setup

        // get a reference to our analog sensor
        //anaLF = hardwareMap.get(AnalogInput.class, "anaRotationLF");
        //srvoLF = hardwareMap.get(Servo.class, "srvoLeftFront");

        asrvoSM[LF]= opMode.hardwareMap.get(Servo.class, strSrvoLFName);
        asrvoSM[LB]= opMode.hardwareMap.get(Servo.class, strSrvoLBName);
        asrvoSM[RF]= opMode.hardwareMap.get(Servo.class, strSrvoRFName);
        asrvoSM[RB]= opMode.hardwareMap.get(Servo.class, strSrvoRBName);

        asrvoSM[LF].setPosition(ZERO_POS[LF]);
        asrvoSM[LB].setPosition(ZERO_POS[LB]);
        asrvoSM[RF].setPosition(ZERO_POS[RF]);
        asrvoSM[RB].setPosition(ZERO_POS[RB]);

        aanaSM[LF]=opMode.hardwareMap.get(AnalogInput.class, strAnaLFName);
        aanaSM[LB]=opMode.hardwareMap.get(AnalogInput.class, strAnaLBName);
        aanaSM[RF]=opMode.hardwareMap.get(AnalogInput.class, strAnaRFName);
        aanaSM[RB]=opMode.hardwareMap.get(AnalogInput.class, strAnaRBName);
    }

    /**
     * Reset the motors for teleop
     *
     * @param bUseEncoders true=use encoders, false= don't use encoders
     */
    public void resetTeleop(boolean bUseEncoders) {
        /*4 motors, so must sync so use encoders, do NOT stop and reset or will not run*/
        int nIdx;
        for (nIdx = 0; nIdx < 4; nIdx++) {
            amtrSM[nIdx].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if(bUseEncoders) {
                amtrSM[nIdx].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                amtrSM[nIdx].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }else {
                amtrSM[nIdx].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        resetOdo();
    }

    /**
     * Reset the motors for autonomous
     * @param bUseEncoders true=use encoders, false= don't use e
     */

    public void resetAuton(boolean bUseEncoders) {
        /*4 motors, so must sync so use encoders, do NOT stop and reset or will not run*/
        int nIdx;
        for (nIdx = 0; nIdx < 4; nIdx++) {
            amtrSM[nIdx].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if(bUseEncoders) {
                amtrSM[nIdx].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                amtrSM[nIdx].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }else {
                amtrSM[nIdx].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        resetOdo();
    }//end  resetAuton

    /**
     * Check if servo position of specific swerve module has been reached
     * @param dPolar target position of servo
     * @param nIdx specific swerve module
     * @return
     */
    boolean chkServoPosFromPolar(double dPolar, int nIdx) {
///wait until servos get to correct position
        double dProportion,dCalc,dLimitHigh,dLimitLow;
        double dServoVolts=aanaSM[nIdx].getVoltage();

        //nIdx=LF, RF, LB, RB
        if((dPolar>=0d)&&(dPolar<90d)) {
            dProportion=(dPolar-0d)/90d; //this will be positive
            dCalc=NINETY_VOLTS[nIdx]-ZERO_VOLTS[nIdx]; //this will be positive
            dCalc=dCalc*dProportion; //this will be positive
            dLimitHigh=ZERO_VOLTS_HIGH[nIdx]+dCalc;
            dLimitLow=ZERO_VOLTS_LOW[nIdx]+dCalc;
            if((dServoVolts>=dLimitLow)&&(dServoVolts<=dLimitHigh)) return true;
            else return false;
        } else if((dPolar>=90d)&&(dPolar<180d)) {
            //assume can't get to 180, so have to use the same proportion as 0 to 90
            dProportion=(dPolar-90d)/90d; //this will be positive
            dCalc=NINETY_VOLTS[nIdx]-ZERO_VOLTS[nIdx]; //this will be positive
            dCalc=dCalc*dProportion; //this will be positive
            dLimitHigh=NINETY_VOLTS_HIGH[nIdx]+dCalc;
            dLimitLow=NINETY_VOLTS_LOW[nIdx]+dCalc;
            if((dServoVolts>=dLimitLow)&&(dServoVolts<=dLimitHigh)) return true;
            else return false;
        } else if((dPolar<0d)&&(dPolar>=-90d)) {
            dProportion=(0d-dPolar)/90d; //this will be positive
            dCalc=ZERO_VOLTS[nIdx]-NEG_NINETY_VOLTS[nIdx]; //this will be positive
            dCalc=dCalc*dProportion; //this will be positive
            dLimitHigh=NEG_NINETY_VOLTS_HIGH[nIdx]+dCalc;
            dLimitLow=NEG_NINETY_VOLTS_LOW[nIdx]+dCalc;
            if((dServoVolts>=dLimitLow)&&(dServoVolts<=dLimitHigh)) return true;
            else return false;
        } else if((dPolar<-90d)&&(dPolar>=-180)) {
            //assume can't get to -180, so have to use the same proportion as 0 to -90
            dProportion=(-90d-dPolar)/90d; //this will be positive
            dCalc=ZERO_VOLTS[nIdx]-NEG_NINETY_VOLTS[nIdx]; //this will be positive
            dCalc=dCalc*dProportion; //this will be positive
            dLimitHigh=NEG_NINETY_VOLTS_HIGH[nIdx]+dCalc;
            dLimitLow=NEG_NINETY_VOLTS_LOW[nIdx]+dCalc;
            if((dServoVolts>=dLimitLow)&&(dServoVolts<=dLimitHigh)) return true;
            else return false;
        } else {
            //dPolar is out of range
            return false;
        }
    }
    /**
     * Calculate servo position of specific swerve module from a polar angle
     * @param dPolar Polar angle to convert
     * @param nIdx specific serve module to do conversion
     * @return
     */
    public double calcPosFromPolar(double dPolar,int nIdx) {
        //dPolar of (position pos ninety) comes from stick right or crawl right, wheel position is less than position 0
        //dPolar of (position neg ninety) comes from stick left or crawl left,  wheel position is more than position 0
        double dProportion,dCalc;

        //nIdx=LF, RF, LB, RB
        if((dPolar>=0d)&&(dPolar<90d)) {
            dProportion=(dPolar-0d)/90d; //this will be positive
            dCalc=ZERO_POS[nIdx]-NINETY_POS[nIdx]; //this will be positive
            dCalc=dCalc*dProportion; //this will be positive
            return ZERO_POS[nIdx]-dCalc;
        } else if((dPolar>=90d)&&(dPolar<180d)) {
            //assume can't get to 180, so have to use the same proportion as 0 to 90
            dProportion = (dPolar - 90d) / 90d; //this will be positive
            dCalc = ZERO_POS[nIdx] - NINETY_POS[nIdx]; //this will be positive
            dCalc = dCalc * dProportion; //this will be positive
            return NINETY_POS[nIdx] - dCalc;
        } else if((dPolar<0d)&&(dPolar>=-90d)) {
            dProportion = (0d-dPolar) / 90d; //this will be positive
            dCalc = NEG_NINETY_POS[nIdx]-ZERO_POS[nIdx]; //this will be positive
            dCalc = dCalc * dProportion; //this will be positive
            return ZERO_POS[nIdx] + dCalc;
        } else if((dPolar<-90d)&&(dPolar>=-180)) {
            //assume can't get to -180, so have to use the same proportion as 0 to -90
            dProportion = (-90d-dPolar) / 90d; //this will be positive
            dCalc = NEG_NINETY_POS[nIdx]-ZERO_POS[nIdx]; //this will be positive
            dCalc = dCalc * dProportion; //this will be positive
            return NEG_NINETY_POS[nIdx] + dCalc;

        } else {
            //dPolar is out of range
            return .5;// a neutral number;
        }

    }

    /**
     * Calc polar angle when given an X and y correction request
     * @param dOppX the X value to correct.  Left is negative
     * @param dAdjY the Y value to correct.
     * @return
     */
    public double calcPolarXYReq( double dOppX, double dAdjY)  {
        double dTheta;
        double dPolar=0;

        if(dOppX==0) dTheta=90d;
        else dTheta=Math.toDegrees(Math.atan2(Math.abs(dOppX),Math.abs(dAdjY)));

        if((dOppX>=0)&&(dAdjY>=0)) {
            dPolar=dTheta;
        } else if((dOppX<0)&&(dAdjY>=0)) {
            dPolar=-dTheta;
        } else if((dOppX>=0)&&(dAdjY<0)) {
            dPolar = dTheta;
        } else if((dOppX<0)&&(dAdjY<0)) {
            dPolar=-dTheta;

        }
        return dPolar;

    }

    /**
     * rotate servo to a polar postion oiven an X and y correction request
     * @param linearOpMode
     * @param dOppX x value to correct.  left is negative
     * @param dAdjY y value to correct
     * @return
     */
    public boolean rotateServoToXYReq(LinearOpMode linearOpMode,double dOppX, double dAdjY, long lTimeout) {
        return rotateServoToPosLOp(linearOpMode,calcPolarXYReq(dOppX,dAdjY),lTimeout);
    }

    /**
     * Rotate swerve module servos to a polar position
     *    linear op mode, so a while loop is involved
     * @param linopMode
     * @param dPolar polar to move to
     * @param lTimeout max time to wait for servos to get to 0
     * @return true if all servos got to position
     */
    public boolean rotateServoToPosLOp(LinearOpMode linopMode, double dPolar, long lTimeout) {
        int nIdx;
        double dPos;
        long lTimestamp=System.currentTimeMillis();
        boolean bAllOkay;
        boolean[] abChk= new boolean[4];
        double[] adVoltage= new double[4];
        double[] adPos= new double[4];
        //set servos moving

        for(nIdx=0;nIdx<=3;nIdx++) {
            dPos=calcPosFromPolar(dPolar,nIdx);
            asrvoSM[nIdx].setPosition(dPos);
        }
        //wait until servos get to correct position
        while((linopMode.opModeIsActive())&&((System.currentTimeMillis()-lTimestamp)<=lTimeout)) {
            bAllOkay=true;
            for(nIdx=0;nIdx<4;nIdx++) {
                if((abChk[nIdx]=chkServoPosFromPolar(asrvoSM[nIdx].getPosition(),nIdx))==false)
                    bAllOkay=false;
                adPos[nIdx]=asrvoSM[nIdx].getPosition();
                adVoltage[nIdx]=aanaSM[nIdx].getVoltage();
                linopMode.telemetry.addData(gstrClassName,"%s %d targetPosition:%.2f volt:%.2f",
                        abChk[nIdx],nIdx,asrvoSM[nIdx].getPosition(),aanaSM[nIdx].getVoltage());
            }
            //linopMode.telemetry.update();
            if(bAllOkay) return true;
        }
        //timed out
        return false;
    }

    /**
     * Rotate swerve module servos to rotating position
     *    linear op mode, so a while loop is involved
     * @param linopMode
     * @param lTimeout max time to wait for servos to get to 0
     * @return true if all servos got to position
     */
    public boolean rotateServoToSpinPosLOp(LinearOpMode linopMode, long lTimeout) {
        int nIdx;
        double dPos;
        long lTimestamp=System.currentTimeMillis();
        boolean bAllOkay;
        boolean[] abChk= new boolean[4];
        double[] adVoltage= new double[4];
        double[] adPos= new double[4];
        //set servos moving


        asrvoSM[LF].setPosition(calcPosFromPolar(45,LF));
        asrvoSM[RF].setPosition(calcPosFromPolar(-45,RF));
        asrvoSM[LB].setPosition(calcPosFromPolar(-45,LB));
        asrvoSM[RB].setPosition(calcPosFromPolar(45,RB));
        //wait until servos get to correct position
        while((System.currentTimeMillis()-lTimestamp)<=lTimeout) {
            bAllOkay=true;
            for(nIdx=0;nIdx<4;nIdx++) {
                if((abChk[nIdx]=chkServoPosFromPolar(asrvoSM[nIdx].getPosition(),nIdx))==false)
                    bAllOkay=false;
                adPos[nIdx]=asrvoSM[nIdx].getPosition();
                adVoltage[nIdx]=aanaSM[nIdx].getVoltage();
                linopMode.telemetry.addData(gstrClassName,"%s %d targetPosition:%.2f volt:%.2f",
                        abChk[nIdx],nIdx,asrvoSM[nIdx].getPosition(),aanaSM[nIdx].getVoltage());
            }
            //linopMode.telemetry.update();
            if(bAllOkay) return true;
        }
        //timed out
        return false;
    }

    /**
     * Rotate swerve module servos to polar 90
     *    linear op mode, so a while loop is involved
     * @param linopMode
     * @param lTimeout max time to wait for servos to get to 90
     * @return true if all servos got to position
     */
    public boolean rotateServoToNegPolar90LOp(LinearOpMode linopMode, long lTimeout) {
        //set servos moving
        asrvoSM[LF].setPosition(NEG_NINETY_POS[LF]);
        asrvoSM[LB].setPosition(NEG_NINETY_POS[LB]);
        asrvoSM[RF].setPosition(NEG_NINETY_POS[RF]);
        asrvoSM[RB].setPosition(NEG_NINETY_POS[RB]);
        long lTimestamp=System.currentTimeMillis();

        //wait until servos get to correct position
        while((System.currentTimeMillis()-lTimestamp)<=lTimeout) {
            //get voltage of servos
            adServoVolts[LF]=aanaSM[LF].getVoltage();
            adServoVolts[LB]=aanaSM[LB].getVoltage();
            adServoVolts[RF]=aanaSM[RF].getVoltage();
            adServoVolts[RB]=aanaSM[RB].getVoltage();
            if((adServoVolts[LF]<=NEG_NINETY_VOLTS_HIGH[LF])&&(adServoVolts[LF]>=NEG_NINETY_VOLTS_LOW[LF])&&
                    (adServoVolts[LB]<=NEG_NINETY_VOLTS_HIGH[LB])&&(adServoVolts[LB]>=NEG_NINETY_VOLTS_LOW[LB])&&
                    (adServoVolts[RF]<=NEG_NINETY_VOLTS_HIGH[RF])&&(adServoVolts[RF]>=NEG_NINETY_VOLTS_LOW[RF])&&
                    (adServoVolts[RB]<=NEG_NINETY_VOLTS_HIGH[RB])&&(adServoVolts[RB]>=NEG_NINETY_VOLTS_LOW[RB])){
                //if got here, all rotations complete
                return true;
            }
        }
        //timed out
        return false;
    }

    /**
     * Rotate swerve module servos to polar 90
     *    op mode, so immediate return
     * @param OpMode
     * @return true if all servos got to position
     */
    public boolean rotateServoToNegPolar90Op(OpMode opMode) {
        //set servos moving
        asrvoSM[LF].setPosition(NEG_NINETY_POS[LF]);
        asrvoSM[LB].setPosition(NEG_NINETY_POS[LB]);
        asrvoSM[RF].setPosition(NEG_NINETY_POS[RF]);
        asrvoSM[RB].setPosition(NEG_NINETY_POS[RB]);

        //check if servos are in correct position
        //get voltage of servos
        adServoVolts[LF]=aanaSM[LF].getVoltage();
        adServoVolts[LB]=aanaSM[LB].getVoltage();
        adServoVolts[RF]=aanaSM[RF].getVoltage();
        adServoVolts[RB]=aanaSM[RB].getVoltage();
        if((adServoVolts[LF]<=NEG_NINETY_VOLTS_HIGH[LF])&&(adServoVolts[LF]>=NEG_NINETY_VOLTS_LOW[LF])&&
                (adServoVolts[LB]<=NEG_NINETY_VOLTS_HIGH[LB])&&(adServoVolts[LB]>=NEG_NINETY_VOLTS_LOW[LB])&&
                (adServoVolts[RF]<=NEG_NINETY_VOLTS_HIGH[RF])&&(adServoVolts[RF]>=NEG_NINETY_VOLTS_LOW[RF])&&
                (adServoVolts[RB]<=NEG_NINETY_VOLTS_HIGH[RB])&&(adServoVolts[RB]>=NEG_NINETY_VOLTS_LOW[RB])){
            //if got here, all rotations complete
            return true;
        }
        //timed out
        return false;
    }
    /**
     * Rotate swerve module servos to polar 0
     *    linear op mode, so a while loop is involved
     * @param linopMode
     * @param lTimeout max time to wait for servos to get to 0
     * @return true if all servos got to position
     */
    public boolean rotateServoToPolar0LOp(LinearOpMode linopMode, long lTimeout) {
        long lTimestamp=System.currentTimeMillis();
        //set servos moving
        asrvoSM[LF].setPosition(ZERO_POS[LF]);
        asrvoSM[LB].setPosition(ZERO_POS[LB]);
        asrvoSM[RF].setPosition(ZERO_POS[RF]);
        asrvoSM[RB].setPosition(ZERO_POS[RB]);
        //wait until servos get to correct position
        while((System.currentTimeMillis()-lTimestamp)<=lTimeout) {
            //get voltage of servos
            adServoVolts[LF]=aanaSM[LF].getVoltage();
            adServoVolts[LB]=aanaSM[LB].getVoltage();
            adServoVolts[RF]=aanaSM[RF].getVoltage();
            adServoVolts[RB]=aanaSM[RB].getVoltage();
            if((adServoVolts[LF]<=ZERO_VOLTS_HIGH[LF])&&(adServoVolts[LF]>=ZERO_VOLTS_LOW[LF])&&
                    (adServoVolts[LB]<=ZERO_VOLTS_HIGH[LB])&&(adServoVolts[LB]>=ZERO_VOLTS_LOW[LB])&&
                    (adServoVolts[RF]<=ZERO_VOLTS_HIGH[RF])&&(adServoVolts[RF]>=ZERO_VOLTS_LOW[RF])&&
                    (adServoVolts[RB]<=ZERO_VOLTS_HIGH[RB])&&(adServoVolts[RB]>=ZERO_VOLTS_LOW[RB])){
                //if got here, all rotations complete
                return true;
            }
        }
        //timed out
        return false;
    }

    public void calibrateServoLOp(LinearOpMode linOpMode,int nPos,long lCalibrationTime) {
        long lTimestamp=System.currentTimeMillis();
        long lCount=0;
        int nIdx=0,nAvgIdx;
        double[] adLFVolt = new double[5]; double[] adRFVolt = new double[5];
        double[] adLBVolt = new double[5]; double[] adRBVolt = new double[5];
        double[] adAvgVolt = new double[4];
        double[] adTotalVolt = new double[4];

        //set servos moving
        if(nPos==0) {
            asrvoSM[LF].setPosition(ZERO_POS[LF]); asrvoSM[RF].setPosition(ZERO_POS[RF]);
            asrvoSM[LB].setPosition(ZERO_POS[LB]); asrvoSM[RB].setPosition(ZERO_POS[RB]);
        } else if(nPos==90) {
            asrvoSM[LF].setPosition(NINETY_POS[LF]); asrvoSM[RF].setPosition(NINETY_POS[RF]);
            asrvoSM[LB].setPosition(NINETY_POS[LB]); asrvoSM[RB].setPosition(NINETY_POS[RB]);
        } else if(nPos==-90) {
            asrvoSM[LF].setPosition(NEG_NINETY_POS[LF]); asrvoSM[RF].setPosition(NEG_NINETY_POS[RF]);
            asrvoSM[LB].setPosition(NEG_NINETY_POS[LB]); asrvoSM[RB].setPosition(NEG_NINETY_POS[RB]);
        } else if(nPos==180) {
            asrvoSM[LF].setPosition(ONEEIGHTY_POS[LF]); asrvoSM[RF].setPosition(ONEEIGHTY_POS[RF]);
            asrvoSM[LB].setPosition(ONEEIGHTY_POS[LB]); asrvoSM[RB].setPosition(ONEEIGHTY_POS[RB]);
        } else {
            linOpMode.telemetry.addData(gstrClassName,"incorrect position. Use 0,90,-90,180");
            linOpMode.telemetry.update();
            return;
        }

        //display telemetry for lCalibrationTime
        while((System.currentTimeMillis()-lTimestamp)<=lCalibrationTime) {
            //get current voltage of servos
            adServoVolts[LF]=aanaSM[LF].getVoltage(); adServoVolts[RF]=aanaSM[RF].getVoltage();
            adServoVolts[LB]=aanaSM[LB].getVoltage(); adServoVolts[RB]=aanaSM[RB].getVoltage();

            //store current for avg calc
            adLFVolt[nIdx]=adServoVolts[LF]; adRFVolt[nIdx]=adServoVolts[RF];
            adLBVolt[nIdx]=adServoVolts[LB]; adRBVolt[nIdx]=adServoVolts[RB];
            nIdx++;
            if(nIdx==5) nIdx=0; //rolling 5
            lCount++; //total times

            //avgs
            if(lCount>=5) {
                adTotalVolt[LF]=adTotalVolt[LB]=adTotalVolt[RF]=adTotalVolt[RB]=0;
                for(nAvgIdx=0;nAvgIdx<5;nAvgIdx++) {
                    adTotalVolt[LF]+=adLFVolt[nAvgIdx]; adTotalVolt[RF]+=adRFVolt[nAvgIdx];
                    adTotalVolt[LB]+=adLBVolt[nAvgIdx]; adTotalVolt[RB]+=adRBVolt[nAvgIdx];
                }
                adAvgVolt[LF]=adTotalVolt[LF]/(double)5; adAvgVolt[RF]=adTotalVolt[RF]/(double)5;
                adAvgVolt[LB]=adTotalVolt[LF]/(double)5; adAvgVolt[RB]=adTotalVolt[RB]/(double)5;
            } else {
                //not enough to make an average
                linOpMode.telemetry.addData(gstrClassName,"not enough data for calibration");
                //linOpMode.telemetry.update();
                linOpMode.sleep(20);
                continue;
            }

            linOpMode.telemetry.addData(gstrClassName,"Servo Pos %d  ",nPos);
            if(nPos==0) {
                linOpMode.telemetry.addData(gstrClassName, "   LF:%.2f %.2fV Avg",
                       ZERO_POS[LF], adAvgVolt[LF]);
                linOpMode.telemetry.addData(gstrClassName, "   LB:%.2f %.2fV Avg",
                       ZERO_POS[LB], adAvgVolt[LB]);
                linOpMode.telemetry.addData(gstrClassName, "   RF:%.2f %.2fV Avg",
                       ZERO_POS[RF], adAvgVolt[RF]);
                linOpMode.telemetry.addData(gstrClassName, "   RB:%.2f %.2fV Avg",
                       ZERO_POS[RB], adAvgVolt[RB]);
            } else if(nPos==90) {
                linOpMode.telemetry.addData(gstrClassName, "   LF:%.2f %.2fV Avg",
                        NINETY_POS[LF], adAvgVolt[LF]);
                linOpMode.telemetry.addData(gstrClassName, "   LB:%.2f %.2fV Avg",
                        NINETY_POS[LB], adAvgVolt[LB]);
                linOpMode.telemetry.addData(gstrClassName, "   RF:%.2f %.2fV Avg",
                        NINETY_POS[RF], adAvgVolt[RF]);
                linOpMode.telemetry.addData(gstrClassName, "   RB:%.2f %.2fV Avg",
                        NINETY_POS[RB], adAvgVolt[RB]);
            } else if(nPos==-90) {
                linOpMode.telemetry.addData(gstrClassName, "   LF:%.2f %.2fV Avg",
                    NEG_NINETY_POS[LF], adAvgVolt[LF]);
                linOpMode.telemetry.addData(gstrClassName, "   LB:%.2f %.2fV Avg",
                    NEG_NINETY_POS[LB], adAvgVolt[LB]);
                linOpMode.telemetry.addData(gstrClassName, "   RF:%.2f %.2fV Avg",
                    NEG_NINETY_POS[RF], adAvgVolt[RF]);
                linOpMode.telemetry.addData(gstrClassName, "   RB:%.2f %.2fV Avg",
                    NEG_NINETY_POS[RB], adAvgVolt[RB]);
            } else if(nPos==180) {
                linOpMode.telemetry.addData(gstrClassName, "   LF:%.2f %.2fV Avg",
                    ONEEIGHTY_POS[LF], adAvgVolt[LF]);
                linOpMode.telemetry.addData(gstrClassName, "   LB:%.2f %.2fV Avg",
                    ONEEIGHTY_POS[LB], adAvgVolt[LB]);
                linOpMode.telemetry.addData(gstrClassName, "   RF:%.2f %.2fV Avg",
                    ONEEIGHTY_POS[RF], adAvgVolt[RF]);
                linOpMode.telemetry.addData(gstrClassName, "   RB:%.2f %.2fV Avg",
                    ONEEIGHTY_POS[RB], adAvgVolt[RB]);

            }

            //linOpMode.telemetry.update();
            linOpMode.sleep(20);
        } //end while

    }

    /**
     * Rotate swerve module servos to polar 0
     *    op mode so immediate return of status,
     * @param opMode
     * @return true if all servos got to position
     */
    public boolean rotateServoToPolar0Op(OpMode opMode) {
        long lTimestamp=System.currentTimeMillis();
        //set servos moving
        asrvoSM[LF].setPosition(ZERO_POS[LF]);
        asrvoSM[LB].setPosition(ZERO_POS[LB]);
        asrvoSM[RF].setPosition(ZERO_POS[RF]);
        asrvoSM[RB].setPosition(ZERO_POS[RB]);
        //check if are in correct position
        //get voltage of servos
        adServoVolts[LF]=aanaSM[LF].getVoltage();
        adServoVolts[LB]=aanaSM[LB].getVoltage();
        adServoVolts[RF]=aanaSM[RF].getVoltage();
        adServoVolts[RB]=aanaSM[RB].getVoltage();
        if((adServoVolts[LF]<=ZERO_VOLTS_HIGH[LF])&&(adServoVolts[LF]>=ZERO_VOLTS_LOW[LF])&&
                (adServoVolts[LB]<=ZERO_VOLTS_HIGH[LB])&&(adServoVolts[LB]>=ZERO_VOLTS_LOW[LB])&&
                (adServoVolts[RF]<=ZERO_VOLTS_HIGH[RF])&&(adServoVolts[RF]>=ZERO_VOLTS_LOW[RF])&&
                (adServoVolts[RB]<=ZERO_VOLTS_HIGH[RB])&&(adServoVolts[RB]>=ZERO_VOLTS_LOW[RB])){
            //if got here, all rotations complete
            return true;
        }
        //got to correct position
        return false;
    }


    /**
     * Rotate swerve module servos to polar 180
     *    Inaccurate!  180 cant be done by Axon Max
     *    linear op mode, so a while loop is involved
     * @param linopMode
     * @param lTimeout max time to wait for servos to get to polar 90
     * @return true if all servos got to position
     */
    public boolean rotateServoToPolar180LOp(LinearOpMode linopMode, long lTimeout) {
        long lTimestamp=System.currentTimeMillis();
        //set servos moving
        asrvoSM[LF].setPosition(ONEEIGHTY_POS[LF]);
        asrvoSM[LB].setPosition(ONEEIGHTY_POS[LB]);
        asrvoSM[RF].setPosition(ONEEIGHTY_POS[RF]);
        asrvoSM[RB].setPosition(ONEEIGHTY_POS[RB]);
        //wait until servos get to correct position

        while((System.currentTimeMillis()-lTimestamp)<=lTimeout) {
            //get voltage of servos
            adServoVolts[LF]=aanaSM[LF].getVoltage();
            adServoVolts[LB]=aanaSM[LB].getVoltage();
            adServoVolts[RF]=aanaSM[RF].getVoltage();
            adServoVolts[RB]=aanaSM[RB].getVoltage();
            if((adServoVolts[LF]<=ONEEIGHTY_VOLTS_HIGH[LF])&&(adServoVolts[LF]>=ONEEIGHTY_VOLTS_LOW[LF])&&
                    (adServoVolts[LB]<=ONEEIGHTY_VOLTS_HIGH[LB])&&(adServoVolts[LB]>=ONEEIGHTY_VOLTS_LOW[LB])&&
                    (adServoVolts[RF]<=ONEEIGHTY_VOLTS_HIGH[RF])&&(adServoVolts[RF]>=ONEEIGHTY_VOLTS_LOW[RF])&&
                    (adServoVolts[RB]<=ONEEIGHTY_VOLTS_HIGH[RB])&&(adServoVolts[RB]>=ONEEIGHTY_VOLTS_LOW[RB])){
                //if got here, all rotations complete
                return true;
            }
        }

        //timed out
        return false;
    }
    /**
     * Rotate swerve module servos to polar 180
     *    Inaccurate!  180 cant be done by Axon Max
     *    linear op mode, so a while loop is involved
     * @param linopMode
     * @return true if all servos got to position
     */
    public boolean rotateServoToPolar180Op(OpMode opMode) {
        //set servos moving
        asrvoSM[LF].setPosition(ONEEIGHTY_POS[LF]);
        asrvoSM[LB].setPosition(ONEEIGHTY_POS[LB]);
        asrvoSM[RF].setPosition(ONEEIGHTY_POS[RF]);
        asrvoSM[RB].setPosition(ONEEIGHTY_POS[RB]);
        //check if servos are in correct position

        //get voltage of servos
        adServoVolts[LF]=aanaSM[LF].getVoltage();
        adServoVolts[LB]=aanaSM[LB].getVoltage();
        adServoVolts[RF]=aanaSM[RF].getVoltage();
        adServoVolts[RB]=aanaSM[RB].getVoltage();
        if((adServoVolts[LF]<=ONEEIGHTY_VOLTS_HIGH[LF])&&(adServoVolts[LF]>=ONEEIGHTY_VOLTS_LOW[LF])&&
                (adServoVolts[LB]<=ONEEIGHTY_VOLTS_HIGH[LB])&&(adServoVolts[LB]>=ONEEIGHTY_VOLTS_LOW[LB])&&
                (adServoVolts[RF]<=ONEEIGHTY_VOLTS_HIGH[RF])&&(adServoVolts[RF]>=ONEEIGHTY_VOLTS_LOW[RF])&&
                (adServoVolts[RB]<=ONEEIGHTY_VOLTS_HIGH[RB])&&(adServoVolts[RB]>=ONEEIGHTY_VOLTS_LOW[RB])){
            //if got here, all rotations complete
            return true;
        }

        //timed out
        return false;
    }

    /**
     * Rotate swerve module servos to polar 90
     *    linear op mode, so a while loop is involved
     * @param linopMode
     * @param lTimeout max time to wait for servos to get to polar 90
     * @return true if all servos got to position
     */
    public boolean rotateServoToPolar90LOp(LinearOpMode linopMode, long lTimeout) {
        //set servos moving
        asrvoSM[LF].setPosition(NINETY_POS[LF]);
        asrvoSM[LB].setPosition(NINETY_POS[LB]);
        asrvoSM[RF].setPosition(NINETY_POS[RF]);
        asrvoSM[RB].setPosition(NINETY_POS[RB]);
        long lTimestamp=System.currentTimeMillis();

        //wait until servos get to correct position
        while((System.currentTimeMillis()-lTimestamp)<=lTimeout) {
            //get voltage of servos
            adServoVolts[LF]=aanaSM[LF].getVoltage();
            adServoVolts[LB]=aanaSM[LB].getVoltage();
            adServoVolts[RF]=aanaSM[RF].getVoltage();
            adServoVolts[RB]=aanaSM[RB].getVoltage();
            linopMode.telemetry.addData(gstrClassName,"LF:%.2f RF:%.2f LF:%.2f LB:%2f",
                    adServoVolts[LF],
                    adServoVolts[RF],
                    adServoVolts[LB],
                    adServoVolts[RB]
            );
            //linopMode.telemetry.update();
            if((adServoVolts[LF]<=NINETY_VOLTS_HIGH[LF])&&(adServoVolts[LF]>=NINETY_VOLTS_LOW[LF])&&
                    (adServoVolts[LB]<=NINETY_VOLTS_HIGH[LB])&&(adServoVolts[LB]>=NINETY_VOLTS_LOW[LB])&&
                    (adServoVolts[RF]<=NINETY_VOLTS_HIGH[RF])&&(adServoVolts[RF]>=NINETY_VOLTS_LOW[RF])&&
                    (adServoVolts[RB]<=NINETY_VOLTS_HIGH[RB])&&(adServoVolts[RB]>=NINETY_VOLTS_LOW[RB])){
                //if got here, all rotations complete
                return true;
            }
        }
        //timed out
        return false;
    }

    /**
     * Rotate swerve module servos to polar 90
     *    op mode, so a while loop is involved
     * @param OpMode
     * @return true if all servos got to position
     */
    public boolean rotateServoToPolar90Op(OpMode opMode) {
        //set servos moving
        asrvoSM[LF].setPosition(NINETY_POS[LF]);
        asrvoSM[LB].setPosition(NINETY_POS[LB]);
        asrvoSM[RF].setPosition(NINETY_POS[RF]);
        asrvoSM[RB].setPosition(NINETY_POS[RB]);

        //wait until servos get to correct position
        adServoVolts[LF]=aanaSM[LF].getVoltage();
        adServoVolts[LB]=aanaSM[LB].getVoltage();
        adServoVolts[RF]=aanaSM[RF].getVoltage();
        adServoVolts[RB]=aanaSM[RB].getVoltage();
        /*
        opMode.telemetry.addData(gstrClassName,"LF:%.2f RF:%.2f LF:%.2f LB:%2f",
                    adServoVolts[LF],
                    adServoVolts[RF],
                    adServoVolts[LB],
                    adServoVolts[RB]
            );
        opMode.telemetry.update();
        */
        if((adServoVolts[LF]<=NINETY_VOLTS_HIGH[LF])&&(adServoVolts[LF]>=NINETY_VOLTS_LOW[LF])&&
                (adServoVolts[LB]<=NINETY_VOLTS_HIGH[LB])&&(adServoVolts[LB]>=NINETY_VOLTS_LOW[LB])&&
                (adServoVolts[RF]<=NINETY_VOLTS_HIGH[RF])&&(adServoVolts[RF]>=NINETY_VOLTS_LOW[RF])&&
                (adServoVolts[RB]<=NINETY_VOLTS_HIGH[RB])&&(adServoVolts[RB]>=NINETY_VOLTS_LOW[RB])){
            //if got here, all rotations complete
            return true;
        }
        //not at correct position
        return false;
    }




}//end class

class VectorPolarV4AS {
    private double x=0,y=0;
    private boolean gbDebug=false;

    private double gdPolar=0,gdPolarOptimal=0,gdPolarToUse=0;
    private double gdMagnitude=0,gdMagnitudeOptimal=0,gdMagnitudeToUse=0;
    private double gdPolarPrev=0,gdMagnitudePrev=0;

    VectorPolarV4AS() {
        
    }

    /**
     * get the vector's x cartesian value
     * @return x
     */
    public double getX() {
        return x;
    }

    /**
     * get the vector's y cartesian value
     * @return y 
     */
    public double getY() {
        return y;
    }

    /**
     * get the magnitude of the vector/
     *
     * @return magnitue of vector
     */
    public double getMagnitude() {
        return gdMagnitude;
    }

    /**
     * get the calculated magnitude of the vector/
     *
     * @return calculated magnitue of vector
     */
    public double calcMagnitude() {
        double dMagnitude= Math.sqrt((x * x) + (y * y));
        return dMagnitude;
    }

    /**
     * get the optimal  magnitude for the optimal polar angle
     * @return magnitue for the optimal polar angle
     */
    public double getMagnitudeOptimal() {
        return gdMagnitudeOptimal;
    }
    /**
     * get the magnitude to use
     * @return magnitude for the to use Polar Angle
     */
    public double getMagnitudeToUse() {
        return gdMagnitudeToUse;
    }
    /**
     * set the magnitude to use
     * @param dMagnitude magnitude to set
     */
    public void setMagnitudeToUse(double dMagnitude) {
        gdMagnitudePrev=gdMagnitudeToUse;
        gdMagnitudeToUse=dMagnitude;
    }


    /**
     * get the polar angle stored
     * @return polar angle stored
     */
    public double getPolarAngle() {
        return gdPolar;
    }

    /**
     * get the polar angle to use
     * @return polar angle to use
     */
    public double getPolarAngleToUse() {
        return gdPolarToUse;
    }
    /**
     * set the polar angle to use
     * @param dPolar polar angle to set
     */
    public void setPolarAngleToUse(double dPolar) {
        gdPolarPrev=gdPolarToUse;
        gdPolarToUse=dPolar;
    }

    /**
     * get the polar angle used previously
     * @return polar angle used previously
     */
    public double getPolarPrev() {
        return gdPolarPrev;
    }
    /**
     * get the magnitude used previously
     * @return magnitude used previously
     */

    /**
     * set the previous polar angle... used for crawling
     * @param dPolar
     * @return
     */
    public void setPolarPrev(double dPolar)  {
       gdPolarPrev=dPolar;
    }
    public double getMagnitudePrev() {
        return gdMagnitudePrev;
    }
    /**
     * get the polar angle at Zero magnitude
     * @return polar angle 0 mag
     */

    /**
     * calc the polar angle of the vector 
     * @return polar angle
     */
    public double calcPolarAngle() {
        double dTheta;
        double dPolar=0;

        if(x==0) dTheta=90d;
        else dTheta=Math.toDegrees(Math.atan2(y, x));
        //        |
        //   \ -162            Recall: Polor North is 0. Degress move clockwise for positive
        //  |  \  |                    Theta East is 0.  Degrees move Counterclockwise for positive
        //--------+======
        //        0
        //      theta = -162
        //       if theta>=0 and theta<=90  say 60
        //           polar=90-theta
        //           polar=90-60 = 30
        //       else if (theta > 90) and (theta<=180), say 100
        //           polar=-[theta-90] =-[100-90] =-10 
        //           polar=-[100-90]=-[10]=-10
        //       else if (theta<0) and (theta>=-90)say -75
        //           polar=-theta+90
        //       else if (theta<-90) and (theta>=-180) say -162
        //           polar=-180-(theta+90)
        //
        if((dTheta>=0d)&&(dTheta<=90d)) dPolar=90d-dTheta;
        else if((dTheta>90d)&&(dTheta<=180)) dPolar=-(dTheta-90d);
        else if((dTheta<0)&&(dTheta>=-90)) dPolar=(90+(-1d*dTheta));
        else if((dTheta<-90)&&(dTheta>=-180)) dPolar=-180-(dTheta+90);
        if(gbDebug) {
            if(gbDebug) System.out.printf("%s.setVectorUsingPolar theta is %.2f polar is %.2f\n",
                this.getClass().getSimpleName(),dTheta,dPolar);
        }

        return dPolar; 
    }

    /**
     * Return the polar angle as an angle between -90 and 90, 
     * MUST use the coresponding getMagnitueOptimal()
     * @return the optimal polar angle MUST use optimal magnitude 
     */
    public double getPolarAngleOptimal() {
        
        return gdPolarOptimal;
    }

    /**
     * set the vector using a polar angle  and magnitude
     * @param dMagnitude magniture of vector
     * @param dPolar polar angle of vector
     * @return
     */
    public void setVectorUsingPolar(double dMagnitude,double dPolar) {
        double dTheta=0;
        //NOTE: need to check the inputted polar        
        dPolar=fixPolar(dPolar);
        //flip polar if magniture is negative
        if(dMagnitude<0) { 
        
            if(gbDebug) {
                if(gbDebug) System.out.printf("%s.setVectorUsingPolar received:pwr %.2f polar %.2f\n",
                    this.getClass().getSimpleName(),dMagnitude,dPolar);
            }
            if((dPolar>=0) && (dPolar<=90)) {
                dPolar=-180+dPolar;  //eg. 30 becomes -150
            } else if((dPolar>90) && (dPolar<=180)) {
                //        |
                //     \  |          Negative puts vector in other direction 
                //   10 \ |                 
                //--------+======
                //        0 \  100
                //        |  \
                dPolar=-180+dPolar; //eg 100 becomes -80
            } else if((dPolar<0) && (dPolar>=-90)) {
                //        |
                //   \ -72|          Negative puts vector in other direction 
                //  |  \  |                    
                //--------+======
                //        0 \
                //        |72 \
                dPolar=180+dPolar; //eg -72 becomes  108
            } else if((dPolar<-90)&&(dPolar>=-180)) {
                dPolar=180+dPolar; //eg -120 becomes 60 
            }
            dMagnitude=-dMagnitude;  //since inverted the angle
            if(gbDebug) {
                if(gbDebug) System.out.printf("%s.setVectorUsingPolar after conversion:pwr %.2f polar %.2f\n",
                    this.getClass().getSimpleName(),dMagnitude,dPolar);
            }
        }

        //now convert polar to theta
        //        |
        //   \ -72|            Recall: Polor North is 0. Degress move clockwise for positive
        //  |  \  |                    Theta East is 0.  Degrees move Counterclockwise for positive
        //--------+======
        //        0
        //       polar = -72 
        //       if polar between -90 and 90
        //           theta=90+(-polar) = 90-(-72) = 162
        //           y= sin(162)=.31  x=cos(162)= -.95
        //       else if (polar > 90), say 100
        //           theta=-[polar-90] =-[100-90] =-10 
        //           y=sin(-10)=-.174 x=cos(-10)=.985
        //       else if (polar < -90) say -110
        //           theta=-180-[90+polar]=-270-polar=-270-(-110)=-160
        //           y=sin(-70)=-.94 x=cos(-70)=.342 
        //
        //

        if(gbDebug) {
            if(gbDebug) System.out.printf("%s.setVectorUsingPolar converting:pwr %.2f polar %.2f\n",
                this.getClass().getSimpleName(),dMagnitude,dPolar);
        }

        if((dPolar<=90)&&(dPolar>=-90)) dTheta=90d-dPolar;
        else if(dPolar>90) dTheta=-(dPolar-90d);
        else if(dPolar<-90) dTheta=-270-dPolar;

        if(gbDebug) {
            if(gbDebug) System.out.printf("%s.setVectorUsingPolar after:pwr %.2f theta %.2f\n",
                this.getClass().getSimpleName(),dMagnitude,dTheta);
        }
        

        //store globals
        x=dMagnitude*Math.cos(Math.toRadians(dTheta));
        y=dMagnitude*Math.sin(Math.toRadians(dTheta));
        gdPolar=dPolar;
        gdMagnitude=dMagnitude;
        if((gdPolar>90d) &&(gdPolar<=180d)){
           gdPolarOptimal= -180d+gdPolar;
        } else if((gdPolar<-90d) &&(gdPolar>=-180d)){
            gdPolarOptimal= gdPolar+180d;
        } else {
            gdPolarOptimal = gdPolar;
        }
        if((gdPolar>90d) &&(gdPolar<=180d)){
            gdMagnitudeOptimal= -gdMagnitude;
        } else if((gdPolar<-90d) &&(gdPolar>=-180d)){
            gdMagnitudeOptimal=-gdMagnitude;
        } else {
            gdMagnitudeOptimal=gdMagnitude;
        }

        if(gbDebug) {
            if(gbDebug) System.out.printf("   x:%.2f y:%.2f\n",x,y);
            if(gbDebug) System.out.printf("   polar set:%.2f polar calc:%.2f\n",dPolar,calcPolarAngle());
        }

         
        
    }
    /**
     * Ensure valid polar angle
     *
     * @param dPolarToFix The polar angle to check
     * @return The valid polar angle
     */
    private double fixPolar(double dPolarToFix) {

        double dRes = dPolarToFix;

        dPolarToFix %= 360.0d; //normalize  to 0 to 360;

        if (dPolarToFix > 180d) {
            //more than 180, so polar is actually negative
            dRes = dPolarToFix - 360d;
        } else if (dPolarToFix < -180d) {
            dRes = dPolarToFix + 360d;
        }
        return dRes;
    }

    /**
     * Turn debugging on or off 
     * @param bDebug true=debug on false= debug off
     */
    public void setDebug(boolean bDebug) {
        gbDebug=bDebug;
    }

    /** 
     *  Set the vector using cartesian coordinates
     * @param xIn
     * @param yIn
     */
    public void setVectorUsingCartesian(double xIn,double yIn) {
        x=xIn;
        y=yIn;
        gdPolar=calcPolarAngle();
        gdMagnitude=calcMagnitude();
        if((gdPolar>90d) &&(gdPolar<=180d)){
            gdPolarOptimal= -180d+gdPolar;
        } else if((gdPolar<-90d) &&(gdPolar>=-180d)){
            gdPolarOptimal= gdPolar+180d;
        } else {
            gdPolarOptimal = gdPolar;
        }
        if((gdPolar>90d) &&(gdPolar<=180d)){
            gdMagnitudeOptimal= -gdMagnitude;
        } else if((gdPolar<-90d) &&(gdPolar>=-180d)){
            gdMagnitudeOptimal=-gdMagnitude;
        } else {
            gdMagnitudeOptimal=gdMagnitude;
        }

    }

    

}
