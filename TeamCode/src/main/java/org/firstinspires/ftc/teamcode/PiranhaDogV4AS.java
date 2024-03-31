/*
Copyright 2021 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *com.qualcomm
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */

public class PiranhaDogV4AS {

    //this class name
    private String gstrClassName=this.getClass().getSimpleName();

    //Declare OpMode members
    private DcMotor mtrTeeth;
    // private Servo srvoPiranhaDogLeft,srvoPiranhaDogRight;
    private Servo srvoJaw;
    // private Servo srvoThroatU;
    // private Servo srvoThroatL;
    // private Servo srvoPaw;
    // private Servo srvoTongue;

    //motor constants
    private static Double TEETH_EAT_PWR=-.85d, TEETH_PUKE_PWR=.5d, TEETH_REST_PWR = 0d;


    //goBildaServer
    //Max PWM Range    500-2500μsec
    //Max PWM Range (Continuous)    900-2100µsec
    //(pulsewidth-500)/2000
    //PiranhaDog servo constants
    //private static double PIRANHADOG_RIGHT_OPEN = 0.55;
    //private static double PIRANHADOG_RIGHT_CLOSE= 0.45d; //(2100-500)/2000
    //private static double PIRANHADOG_LEFT_OPEN=0.45d; //(2100-500)/2000
    //private static double PIRANHADOG_LEFT_CLOSE = 0.55d; //(900-500)/2000
    // private static double TONGUE_INIT = .5d;
    // private static double TONGUE_IN = .6d; //.6
    // private static double TONGUE_OUT = .40d; //.3

    //HSRM9382TH  serv0
    //PWM range 800-2200μsec
    //(pulsewidth-500)/2000
    //
    //srvoJaw is this kind of servo
    //(800-500)/2000 = .15 MIN VALUE
    //(2200-500)/2000 = .85 MAX VALUE
    private static double JAW_OPEN = .8125d;
    private static double JAW_LARGE_BITE = .75d;
    private static double JAW_MEDIUM_BITE = .735d;
    private static double JAW_SMALL_BITE = .725d;
    private static double JAW_SPITA = .6d;
    private static double JAW_SPITB = .4d;
    private static double JAW_STOP = .8d;



    private static Double STICK_DEAD_ZONE=.5;
/*
    private final int CAPSTATE_IDLE=0;
    private final int CAPSTATE_BITING = 1;
    private final int CAPSTATE_RAISING_NECK= 2;
    private final int CAPSTATE_EXTENDING_NECK=3;
    private final int CAP_NECK_EXTEND_POS=2550;
    private long glCapTimeStamp=0;
    private boolean gbCapStarted=false;
    private int gnCapState = CAPSTATE_IDLE;
    */


    public void initialize(OpMode opMode) {

        opMode.telemetry.addData(gstrClassName, "Initializing...");
        //opMode.telemetry.addData(gstrClassName, "    Body must be up");
        //opMode.telemetry.addData(gstrClassName, "    If not, Hit Stop, then re-Init");

        //PiranhaDog Motor

        mtrTeeth = opMode.hardwareMap.get(DcMotor.class, "mtrTeeth");
        mtrTeeth.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrTeeth.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrTeeth.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // mtrTeeth.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrTeeth.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrTeeth.setPower(TEETH_REST_PWR);

        //PiranhaDog Servos
        // srvoPiranhaDogLeft = opMode.hardwareMap.get(Servo.class, "srvoPiranhaDogLeft");
        // srvoPiranhaDogLeft.setPosition(PIRANHADOG_LEFT_OPEN);
        // srvoPiranhaDogRight = opMode.hardwareMap.get(Servo.class, "srvoPiranhaDogRight");
        // srvoPiranhaDogRight.setPosition(PIRANHADOG_RIGHT_OPEN);

        srvoJaw = opMode.hardwareMap.get(Servo.class, "srvoJaw");
        srvoJaw.setPosition(JAW_OPEN);



        // srvoThroatU = opMode.hardwareMap.get(Servo.class, "srvoThroatU");
        // srvoThroatU.setPosition(.5);

        // srvoThroatL = opMode.hardwareMap.get(Servo.class, "srvoThroatL");
        // srvoThroatL.setPosition(.5);

        // srvoPaw = opMode.hardwareMap.get(Servo.class, "srvoPaw");
        // srvoPaw.setPosition(.50); //.52 is slight squeeze NOTE!! pixel point rests against back plate, not pixel straight


        // srvoTongue = opMode.hardwareMap.get(Servo.class, "srvoTongue");
        // srvoTongue.setPosition(TONGUE_INIT);//.6


        opMode.telemetry.addData(gstrClassName, "    Initialized");


    }

    public void operate(OpMode opMode) {
        opMode.telemetry.addData("PiranhaDog","Jaw:%.2f",
                srvoJaw.getPosition());

        // move upper jaw
        if (opMode.gamepad2.dpad_up){
            srvoJaw.setPosition(JAW_OPEN);
            mtrTeeth.setPower(TEETH_PUKE_PWR);
            return;
        }
        else if (opMode.gamepad2.dpad_down){
            srvoJaw.setPosition(JAW_SMALL_BITE);
            mtrTeeth.setPower(TEETH_EAT_PWR);
            return;
        }
        else if (opMode.gamepad2.dpad_left){
            srvoJaw.setPosition(JAW_MEDIUM_BITE);
            mtrTeeth.setPower(TEETH_EAT_PWR);
            return;
        }
        else if (opMode.gamepad2.dpad_right){
            srvoJaw.setPosition(JAW_LARGE_BITE);
            mtrTeeth.setPower(TEETH_EAT_PWR);
            return;
        }

        // srvoTongue.setPosition(TONGUE_IN);
        //intake

        if(opMode.gamepad2.left_bumper) {//intake pixel
            mtrTeeth.setPower(TEETH_EAT_PWR);
            return;
        }
        else if(opMode.gamepad2.left_trigger >= .5){//eject pixel
            mtrTeeth.setPower(TEETH_PUKE_PWR);
            return;
        }
        else {
            srvoJaw.setPosition(JAW_OPEN);
            mtrTeeth.setPower(TEETH_REST_PWR);
        }




    }

    public void autonSpitPixel(LinearOpMode linopMode,long lSpitMSec,long lDroolMSec) {


        srvoJaw.setPosition(JAW_SMALL_BITE);
        linopMode.sleep(1000);
        mtrTeeth.setPower(TEETH_EAT_PWR + .4);
        linopMode.sleep(1000);
        srvoJaw.setPosition(JAW_OPEN);
        mtrTeeth.setPower(0);

        //srvoTongue.setPosition(TONGUE_OUT);
        //linopMode.sleep(500);
        // srvoTongue.setPosition(TONGUE_OUT);
        // srvoThroatU.setPosition(.21);
        // srvoThroatL.setPosition(.21);

        // srvoJaw.setPosition(JAW_SPITA);//eject slowly
        // linopMode.sleep(lSpitMSec);//pixel on mat
        // srvoJaw.setPosition(JAW_STOP);

        // srvoJaw.setPosition(JAW_SPITB);//retract jaw
        // linopMode.sleep(lDroolMSec);//pixel on mat
        // srvoJaw.setPosition(JAW_STOP);
        // srvoTongue.setPosition(TONGUE_IN);

        // srvoThroatU.setPosition(.5);
        // srvoThroatL.setPosition(.5);

    }


    public void shutdown(OpMode opMode) {
        return;
    }

    // public void openPaw(OpMode opMode) {
    //     srvoPaw.setPosition(.42); //.47

    //     return;
    // }
    /*
    public void autonExtendNeckLOp(LinearOpMode linopMode, int nExtendPos) {
        int nNeckCurrPos=mtrGiraffe.getCurrentPosition();
        double dPwr=0d;
        //check if need to extend or contract
        if(nExtendPos>nNeckCurrPos) {
            //need to extend
            dPwr=GIRAFFE_EXTEND_PWR;
            while(nExtendPos>mtrGiraffe.getCurrentPosition()){
                mtrGiraffe.setPower(GIRAFFE_EXTEND_PWR);
            }
            mtrGiraffe.setPower(0d);
        }
        else if (nExtendPos<nNeckCurrPos) {
            dPwr=GIRAFFE_EXTEND_PWR;
            while(nExtendPos<mtrGiraffe.getCurrentPosition()){
                mtrGiraffe.setPower(-GIRAFFE_EXTEND_PWR);
            }
            mtrGiraffe.setPower(0d);
        }
    }

    public void autonNeckDownLOp(LinearOpMode linopMode) {
        srvoGiraffeNeck.setPosition(GIRAFFE_NECK_DOWN);
    }

    public void autonNeckUpLOp(LinearOpMode linopMode) {
        srvoGiraffeNeck.setPosition(GIRAFFE_NECK_UP);
    }

    public void autonOpenMouthLOp(LinearOpMode linopMode) {
        srvoGiraffeMouth.setPosition(GIRAFFE_MOUTH_OPEN);
    }

    public void autonCloseMouthLOp(LinearOpMode linopMode) {
        srvoGiraffeMouth.setPosition(GIRAFFE_MOUTH_CLOSED);
    }
    */
    /*
    private int teleopCap(boolean bButtonPressed) {
        //if got here, the cap button was pressed
        //check if this is the first time pressed
        long lTimeStamp;
        if(bButtonPressed) {
            //starting capping
            glCapTimeStamp=System.currentTimeMillis();
            gnCapState=CAPSTATE_BITING;
            srvoGiraffeMouth.setPosition(GIRAFFE_BITE_ELEMENT);
        }



        switch(gnCapState){
            case CAPSTATE_IDLE:
                return CAPSTATE_IDLE;
            case CAPSTATE_BITING:  //biting
                lTimeStamp=System.currentTimeMillis();
                if((lTimeStamp-glCapTimeStamp)>300) {//300 ms to bite
                   glCapTimeStamp=lTimeStamp;
                   gnCapState=CAPSTATE_RAISING_NECK;
                   return CAPSTATE_RAISING_NECK;

                }
                return CAPSTATE_BITING;
            case CAPSTATE_RAISING_NECK: //raising neck
                srvoGiraffeNeck.setPosition(GIRAFFE_NECK_DOWN); //makes neck level to mat
                gnCapState=CAPSTATE_EXTENDING_NECK;
                return CAPSTATE_EXTENDING_NECK;
            case CAPSTATE_EXTENDING_NECK:
                int nNeckCurrPos=mtrGiraffe.getCurrentPosition();
                double dPwr=0d;
                //check if need to extend or contract
                if(CAP_NECK_EXTEND_POS>nNeckCurrPos) {
                    //need to extend
                    dPwr=GIRAFFE_EXTEND_PWR;
                    mtrGiraffe.setPower(GIRAFFE_EXTEND_PWR);
                    return (CAPSTATE_EXTENDING_NECK);

                }
                //done extending
                mtrGiraffe.setPower(0d);

                gnCapState=CAPSTATE_IDLE;
                gbCapStarted=false;
                return (gnCapState);
            default:
                return (gnCapState);
        }


    }
    */
}