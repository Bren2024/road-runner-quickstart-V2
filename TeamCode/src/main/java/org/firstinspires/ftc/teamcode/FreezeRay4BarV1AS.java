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

public class FreezeRay4BarV1AS {
    
    //this class name
    private String gstrClassName=this.getClass().getSimpleName();
    
    //Declare OpMode members
    private DcMotor mtrFreezeRayLeft, mtrFreezeRayRight;
    private Servo srvoBipodLeft, srvoBipodRight;
    private Servo srvoTrigger;
    
    //control constants
    private static long MS_BETWEEN_PRESSES=100;
    private static long MS_BETWEEN_HEIGHT_REQ=700;
    private static double DEAD_ZONE=.5, MOVE_ZONE=.7;
    
    //motor constants
    //private static double RAY_UP_PWR = .5d;
    //private static double RAY_DN_PWR = -.3d;
    private static double RAY_PWR=.75d;
    


    //goBildaServer
    //Max PWM Range    500-2500μsec
    //Max PWM Range (Continuous)    900-2100µsec
    //(pulsewidth-500)/2000
    //PiranhaDog servo constants
    //private static double PIRANHADOG_RIGHT_OPEN = 0.55;
    //private static double PIRANHADOG_RIGHT_CLOSE= 0.45d; //(2100-500)/2000
    //private static double PIRANHADOG_LEFT_OPEN=0.45d; //(2100-500)/2000
    //private static double PIRANHADOG_LEFT_CLOSE = 0.55d; //(900-500)/2000
    
    
    //HSRM9382TH  server
    //PWM range 800-2200μsec
    //(pulsewidth-500)/2000
    //                                              
     
    private static int RAY_REQ_HOLSTER=0;
    private static int RAY_REQ_UNHOLSTER=1;
    private static int RAY_REQ_ONE=2;
    private static int RAY_REQ_TWO=3;
    private static int RAY_REQ_THREE=4;
    private static int RAY_POS_HOLSTER = 0;
    public static int RAY_POS_UNHOLSTER = 1600;
    //public static int RAY_POS_ONE = 1300;
    public static int RAY_POS_ONE =2000;
    public static int RAY_POS_TWO = 2300;
    public static int RAY_POS_THREE = 2800;
    public static int RAY_POS_AUTO = 1450; //height during auto?

    public static int RAY_POS_AUTO_LIFT = 1700;

    private static long RAY_POS_INCR = 50l;
    private static double BIPOD_LEFT_NEUTRAL = .504d;
    private static double BIPOD_RIGHT_NEUTRAL = .499d; //.498 too high, .504 too low, .501 too low
    private static double TRIGGER_NEUTRAL = .5;
    private static double TRIGGER_CLOSE = TRIGGER_NEUTRAL+.02; //holster +.003 , -.24, -.2, -.15, -.05, +.1, +.05, +.02
    private static double TRIGGER_OPEN=TRIGGER_NEUTRAL+.2; //cock +.027
    

    private long glLastControlPress=System.currentTimeMillis();
    private int gnRayPosReq=RAY_REQ_HOLSTER;
    private boolean gbRayAuto=false, gbRayManual=false;
    private int gnManualTarget=0;
    //private double dTriggerCurrentPos = TRIGGER_NEUTRAL;
    private long glLastHeightPress=glLastControlPress;
    
    public void initialize(OpMode opMode) {
  
        opMode.telemetry.addData(gstrClassName, "Initializing...");
        //opMode.telemetry.addData(gstrClassName, "    Body must be up");
        //opMode.telemetry.addData(gstrClassName, "    If not, Hit Stop, then re-Init");
        
        //initialize variables
        glLastControlPress=System.currentTimeMillis();
        //FreezeRay motors
        mtrFreezeRayLeft = opMode.hardwareMap.get(DcMotor.class, "mtrFreezeRayLeft");
        mtrFreezeRayLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //mtrJetLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        mtrFreezeRayRight = opMode.hardwareMap.get(DcMotor.class, "mtrFreezeRayRight");
        mtrFreezeRayRight.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //mtrJetRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
     
        //Aim Freeze Ray with Bipod
        srvoBipodLeft = opMode.hardwareMap.get(Servo.class, "srvoBipodLeft");
        srvoBipodLeft.setPosition(BIPOD_LEFT_NEUTRAL);
        srvoBipodRight = opMode.hardwareMap.get(Servo.class, "srvoBipodRight");
        srvoBipodRight.setPosition(BIPOD_RIGHT_NEUTRAL);
        //Trigger servo
        srvoTrigger = opMode.hardwareMap.get(Servo.class, "srvoTrigger");
        srvoTrigger.setPosition(TRIGGER_CLOSE);
        
        opMode.telemetry.addData(gstrClassName, "    Initialized");
    }
    
    public void operate(OpMode opMode) {
        
        long lCurrentTime=System.currentTimeMillis();   
        
        //check if it is time to look at controls again
        if((lCurrentTime-glLastControlPress)<MS_BETWEEN_PRESSES) return;
        
        //its time to look at controls again
        
        setFreezeRayHeight(opMode);
        setFreezeRayAim(opMode);
        return;
    }
    
    private void setFreezeRayAim(OpMode opMode){
        long lCurrentTime=System.currentTimeMillis();   
        
        opMode.telemetry.addData(gstrClassName,"Bipod L:%.3f R:%.3f",
            srvoBipodLeft.getPosition(),srvoBipodRight.getPosition());
            
        
        opMode.telemetry.addData(gstrClassName,"Trigger%.2f",
            srvoTrigger.getPosition());
            
                
        
        
        //dont do any of this if not raised
        //if(mtrFreezeRayLeft.getCurrentPosition<(POS_ONE/2)) return;
        if (opMode.gamepad2.y) {
            raiseFreezeRay();
            //glLastControlPress=System.currentTimeMillis();
            
        }
        if (opMode.gamepad2.x) {
            
            srvoTrigger.setPosition(TRIGGER_OPEN);
            glLastControlPress=System.currentTimeMillis();
        }
        if (opMode.gamepad2.a) {
            unholsterFreezeRay();
            srvoTrigger.setPosition(TRIGGER_CLOSE);

            glLastControlPress=System.currentTimeMillis();
        }
        
        if (opMode.gamepad2.b) {
            
            srvoTrigger.setPosition(TRIGGER_CLOSE);
            //dropPixel();
            //glLastControlPress=System.currentTimeMillis();
        }
        if ((opMode.gamepad2.right_stick_button)){
            unholsterFreezeRay();//set servo first
            //gnRayPosReq=RAY_POS_HOLSTER;
            gbRayAuto=true;
            mtrFreezeRayLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtrFreezeRayRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            mtrFreezeRayLeft.setPower(RAY_PWR);
//            mtrFreezeRayRight.setPower(RAY_PWR);
        }
        if ((opMode.gamepad2.right_stick_y > (0.4))&&  //stick back
                (opMode.gamepad2.right_stick_x > (-0.4))&&
                (opMode.gamepad2.right_stick_x < (0.4))) {
            gbRayAuto=false;
            mtrFreezeRayRight.setPower(-0.5);
            mtrFreezeRayLeft.setPower(-0.5);
        }
        else if ((opMode.gamepad2.right_stick_y < (-0.4)) &&  //stick forward
                (opMode.gamepad2.right_stick_x > (-0.4))&&
                (opMode.gamepad2.right_stick_x < (0.4))) {
            gbRayAuto=false;
            mtrFreezeRayRight.setPower(0.75);
            mtrFreezeRayLeft.setPower(0.75);
        }
        else if ((opMode.gamepad2.right_stick_x < (-0.4)) &&  //stick left
                (opMode.gamepad2.right_stick_y > (0.4))) {   //and back
            gbRayAuto=false;
            mtrFreezeRayLeft.setPower(-0.5);
            mtrFreezeRayRight.setPower(0);
        }
        else if ((opMode.gamepad2.right_stick_x > (0.4)) &&  //stick right
                (opMode.gamepad2.right_stick_y > (0.4))) {   //and back
            gbRayAuto=false;
            mtrFreezeRayRight.setPower(-0.5);
            mtrFreezeRayLeft.setPower(0);

        }
        else if ((opMode.gamepad2.right_stick_x < (-0.4)) &&  //stick left
                (opMode.gamepad2.right_stick_y < (-0.4))) {   //and  forward
            gbRayAuto=false;
            mtrFreezeRayLeft.setPower(0.75);
            mtrFreezeRayRight.setPower(0);
        }
        else if ((opMode.gamepad2.right_stick_x > (0.4)) &&  //stick right
                (opMode.gamepad2.right_stick_y < (-0.4))){   //and forward
            gbRayAuto=false;
            mtrFreezeRayRight.setPower(0.75);
            mtrFreezeRayLeft.setPower(0);
        } else if ((gbRayAuto==false)){
            mtrFreezeRayLeft.setPower(0);
            mtrFreezeRayRight.setPower(0);
        }

        return;
    }
    
    // from iris probably to iris: 
    // can we just delete this entirely? need to fix all the resulting errors tho
    private void dropPixel(){
        srvoTrigger.setPosition(srvoTrigger.getPosition()+.0015d);
        double dLeftBipodIncrement=BIPOD_LEFT_NEUTRAL-srvoBipodLeft.getPosition(); //this will be 0 or positive
            dLeftBipodIncrement+=(.000235d*1d);  //so can subtract more
        double dRightBipodIncrement=srvoBipodRight.getPosition()-BIPOD_RIGHT_NEUTRAL;//this will be 0 or positive
            dRightBipodIncrement+=(.0002d*1d); //so can add more
            
        srvoBipodLeft.setPosition(BIPOD_LEFT_NEUTRAL-dLeftBipodIncrement);
        srvoBipodRight.setPosition(BIPOD_RIGHT_NEUTRAL+dRightBipodIncrement);
        
        
        
    }
   
    private void holsterFreezeRay(){
        srvoTrigger.setPosition(TRIGGER_CLOSE);//set servo first
        srvoBipodLeft.setPosition(BIPOD_LEFT_NEUTRAL);
        srvoBipodRight.setPosition(BIPOD_RIGHT_NEUTRAL);
       
    }
    private void unholsterFreezeRay(){
        srvoTrigger.setPosition(TRIGGER_CLOSE);//set servo first
        srvoBipodLeft.setPosition(BIPOD_LEFT_NEUTRAL);
        srvoBipodRight.setPosition(BIPOD_RIGHT_NEUTRAL);

    }
    private void raiseFreezeRay () {
        //to raise, need to make left bipod more neg, and right bipod more positive
        double dLeftBipodIncrement=BIPOD_LEFT_NEUTRAL-srvoBipodLeft.getPosition(); //this will be 0 or positive
            dLeftBipodIncrement+=(.000235d*4d);  //so can subtract more
        double dRightBipodIncrement=srvoBipodRight.getPosition()-BIPOD_RIGHT_NEUTRAL;//this will be 0 or positive
            dRightBipodIncrement+=(.0002d*4d); //so can add more
        
        srvoBipodLeft.setPosition(BIPOD_LEFT_NEUTRAL-dLeftBipodIncrement);
        srvoBipodRight.setPosition(BIPOD_RIGHT_NEUTRAL+dRightBipodIncrement);
        
        //now the trigger

        double dTriggerPos=srvoTrigger.getPosition();
        srvoTrigger.setPosition(dTriggerPos+.0002d*4d);//.009
        
    }
    
    private void setFreezeRayHeight(OpMode opMode) {
        long lCurrentTime=System.currentTimeMillis();   
        
        opMode.telemetry.addData(gstrClassName,"L Pos:%d Tgt:%d Pwr:%.2f",
            mtrFreezeRayLeft.getCurrentPosition(),mtrFreezeRayLeft.getTargetPosition(),
            mtrFreezeRayLeft.getPower());
        
        opMode.telemetry.addData(gstrClassName,"R Pos:%d Tgt:%d Pwr:%.2f",
            mtrFreezeRayRight.getCurrentPosition(),mtrFreezeRayRight.getTargetPosition(),
            mtrFreezeRayRight.getPower());
            
        //opMode.telemetry.addData("Ray Auto","%s Req:%d Lbusy:%s RBusy:%s",
        //    gbFreezeRayAuto,gnFreezeRayPosReq,
        //    mtrFreezeRayLeft.isBusy(),mtrFreezeRayRight.isBusy());

        //check if a preset control was used
        if(chkHeightPreset(opMode)) {
            glLastControlPress=lCurrentTime;
            return;
        }
        
        
        //if got here, no new auto positioning was requested
        //auto positioning could be running
        //manual controls override autopositioning
        //stop autopositioning only if pressed a manual control
        
        //if(chkHeightManual(opMode)) {
        //    glLastControlPress=lCurrentTime;        
        //    return;
        //}
        
        //if got here, no control pressed.
        //autopositioning could stll be running
        //if autopostion is off, turn power off
        
        if(gbRayAuto==false) {
            
            mtrFreezeRayLeft.setPower(0);
            mtrFreezeRayRight.setPower(0);
            return;
        }
        
        return;
    }
    
    
    private boolean chkHeightPreset(OpMode opMode) {
        long lCurrentTime=System.currentTimeMillis();
        
        if((lCurrentTime-glLastHeightPress)<MS_BETWEEN_HEIGHT_REQ) return false;
        
        
        if (opMode.gamepad2.right_bumper) {//request move elevator up
            glLastHeightPress=lCurrentTime;
            if(gnRayPosReq==RAY_POS_HOLSTER) {//currently holstered, so un holster
                unholsterFreezeRay();//set servo first
                gnRayPosReq=RAY_POS_UNHOLSTER;
                gbRayAuto=true;    
                mtrFreezeRayLeft.setTargetPosition(RAY_POS_UNHOLSTER);
                mtrFreezeRayRight.setTargetPosition(RAY_POS_UNHOLSTER);
                mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrFreezeRayLeft.setPower(RAY_PWR);
                mtrFreezeRayRight.setPower(RAY_PWR);
            } else if(gnRayPosReq==RAY_POS_UNHOLSTER) {//currently unholstered, so go to POS1

                unholsterFreezeRay();
                gnRayPosReq=RAY_POS_ONE;
                gbRayAuto=true;    
                mtrFreezeRayLeft.setTargetPosition(RAY_POS_ONE);
                mtrFreezeRayRight.setTargetPosition(RAY_POS_ONE);
                mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrFreezeRayLeft.setPower(RAY_PWR);
                mtrFreezeRayRight.setPower(RAY_PWR);
            } else if (gnRayPosReq==RAY_POS_ONE) {//currently on pos one, go to two
                unholsterFreezeRay();
                gnRayPosReq=RAY_POS_TWO;
                gbRayAuto=true;    
                mtrFreezeRayLeft.setTargetPosition(RAY_POS_TWO);
                mtrFreezeRayRight.setTargetPosition(RAY_POS_TWO);
                mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrFreezeRayLeft.setPower(RAY_PWR);
                mtrFreezeRayRight.setPower(RAY_PWR);
                
                
            } else if (gnRayPosReq==RAY_POS_TWO) {//currently on pos two, go to three
                unholsterFreezeRay();
                gnRayPosReq=RAY_POS_THREE;
                gbRayAuto=true;    
                mtrFreezeRayLeft.setTargetPosition(RAY_POS_THREE);
                mtrFreezeRayRight.setTargetPosition(RAY_POS_THREE);
                mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrFreezeRayLeft.setPower(RAY_PWR);
                mtrFreezeRayRight.setPower(RAY_PWR);
                
                
            } 
            
                if(opMode.gamepad2.left_stick_y < -.5) {
                mtrFreezeRayLeft.setPower(RAY_PWR);
                mtrFreezeRayRight.setPower(RAY_PWR);
            
                //return;
            } 
    
        
            return true;
        }//end req to move elevator up
        
        
        if (opMode.gamepad2.right_trigger > DEAD_ZONE) {//request move elevator down
            glLastHeightPress=lCurrentTime;
            if(gnRayPosReq==RAY_POS_HOLSTER) {//currently holstered, do nothing
            } else if(gnRayPosReq==RAY_POS_UNHOLSTER) {//currently unholstered, move to holster
            
                gnRayPosReq=RAY_POS_HOLSTER;
                gbRayAuto=true;
                unholsterFreezeRay(); //angle up so can clear cross bar when holster
                srvoTrigger.setPosition(TRIGGER_CLOSE); //close basket
            
                mtrFreezeRayLeft.setTargetPosition(RAY_POS_HOLSTER);
                mtrFreezeRayRight.setTargetPosition(RAY_POS_HOLSTER);
                mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrFreezeRayLeft.setPower(RAY_PWR);
                mtrFreezeRayRight.setPower(RAY_PWR);
            } else if(gnRayPosReq==RAY_POS_ONE) {//currently pos one, move to holster
            
                gnRayPosReq=RAY_POS_UNHOLSTER;
                gbRayAuto=true;
                unholsterFreezeRay(); //angle up so can clear cross bar when holster
                srvoTrigger.setPosition(TRIGGER_CLOSE); //close basket
            
                mtrFreezeRayLeft.setTargetPosition(RAY_POS_UNHOLSTER);
                mtrFreezeRayRight.setTargetPosition(RAY_POS_UNHOLSTER);
                mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrFreezeRayLeft.setPower(RAY_PWR);
                mtrFreezeRayRight.setPower(RAY_PWR);
            } else if(gnRayPosReq==RAY_POS_TWO)
                {//move to pos one
                gnRayPosReq=RAY_POS_ONE;
                gbRayAuto=true;
                unholsterFreezeRay();
                srvoTrigger.setPosition(TRIGGER_CLOSE);
            
                mtrFreezeRayLeft.setTargetPosition(RAY_POS_ONE);
                mtrFreezeRayRight.setTargetPosition(RAY_POS_ONE);
                mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrFreezeRayLeft.setPower(RAY_PWR);
                mtrFreezeRayRight.setPower(RAY_PWR);
                
            } else if(gnRayPosReq==RAY_POS_THREE)
                {//move to pos one
                gnRayPosReq=RAY_POS_TWO;
                gbRayAuto=true;
                unholsterFreezeRay();
                srvoTrigger.setPosition(TRIGGER_CLOSE);
            
                mtrFreezeRayLeft.setTargetPosition(RAY_POS_TWO);
                mtrFreezeRayRight.setTargetPosition(RAY_POS_TWO);
                mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrFreezeRayLeft.setPower(RAY_PWR);
                mtrFreezeRayRight.setPower(RAY_PWR);
                
            }
            
            if(opMode.gamepad2.right_stick_y >.5){
                mtrFreezeRayLeft.setPower(-RAY_PWR);
                mtrFreezeRayRight.setPower(-RAY_PWR);
    
                //return;
            } 
            return true;
        }
        
        //if got here, no control pressed
        // EDIT MADE BY JOSH FOR TRANSFER ---------------------------------------------------
        //set servo into holster if not already there
        // if(gnRayPosReq==RAY_POS_HOLSTER)srvoTrigger.setPosition(TRIGGER_CLOSE);
        // ----------------------------------------------------------------------------------
        
        if(gbRayAuto&&!(mtrFreezeRayLeft.isBusy())&&!(mtrFreezeRayRight.isBusy())) {
            //if got here, auto request, and motors not busy
            
            mtrFreezeRayLeft.setPower(0);
            mtrFreezeRayRight.setPower(0);
            mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        
        }
        
        return false;
    }
    
    
    public void autonShootPixel(LinearOpMode linopMode,int nHeight, long lTargetTime,long lMaxTime){
        long lTimestamp=System.currentTimeMillis();
        long lShotStart;
        boolean bAimed=false, bShot=false, bHolstered=false;
        //setup motors
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayLeft.setTargetPosition(nHeight);
        mtrFreezeRayRight.setTargetPosition(nHeight);
        mtrFreezeRayLeft.setPower(RAY_PWR);
        mtrFreezeRayRight.setPower(RAY_PWR);
        //wait for bar to raise
        while (linopMode.opModeIsActive() && !linopMode.isStopRequested() && !bAimed) {
            if ((System.currentTimeMillis() - lTimestamp) > lMaxTime) {
                linopMode.telemetry.addData(gstrClassName, "timeout");
                linopMode.telemetry.update();
                return;
            }
            if(!mtrFreezeRayLeft.isBusy() && !mtrFreezeRayRight.isBusy()) {
                bAimed=true;
            } else if((mtrFreezeRayLeft.getCurrentPosition()>=(nHeight-10)) &&
                (mtrFreezeRayRight.getCurrentPosition()>=(nHeight-10))) {
                bAimed=true;
            
            } else {
                linopMode.telemetry.addData(gstrClassName, "motors busy aiming");
                linopMode.telemetry.update();
            }
        }
        mtrFreezeRayRight.setPower(0);
        mtrFreezeRayLeft.setPower(0);
        if(!bAimed) return;
        //set servos
        srvoBipodLeft.setPosition(BIPOD_LEFT_NEUTRAL-.01d);
        srvoBipodRight.setPosition(BIPOD_RIGHT_NEUTRAL+.01d);
        //srvoTrigger.setPosition(TRIGGER_NEUTRAL+.072);
        
        //wait for shot to hit target
        lShotStart=System.currentTimeMillis();
        while (linopMode.opModeIsActive() && !linopMode.isStopRequested() && !bShot) {
            if ((System.currentTimeMillis() - lTimestamp) > lMaxTime) {
                linopMode.telemetry.addData(gstrClassName, "timeout");
                linopMode.telemetry.update();
                return;
            }
            if((System.currentTimeMillis()-lShotStart)>lTargetTime) {
                 bShot=true;
           } else {
                linopMode.telemetry.addData(gstrClassName, "waiting for shot to hit");
                linopMode.telemetry.update();
            }
        }
        
        mtrFreezeRayRight.setPower(0);
        mtrFreezeRayLeft.setPower(0);
        if(!bShot) {
            return;
        }
        //lower FreezeRay

        unholsterFreezeRay();
        linopMode.sleep(1200);
        
        
        
        //holster freeze ray
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayLeft.setTargetPosition(0);
        mtrFreezeRayRight.setTargetPosition(0);
        mtrFreezeRayLeft.setPower(RAY_PWR/2);
        mtrFreezeRayRight.setPower(RAY_PWR/2);
        //wait for holstering
        while (linopMode.opModeIsActive() && !linopMode.isStopRequested() && !bHolstered) {
            if ((System.currentTimeMillis() - lTimestamp) > lMaxTime) {
                linopMode.telemetry.addData(gstrClassName, "timeout");
                linopMode.telemetry.update();
                return;
            }
            if(!mtrFreezeRayLeft.isBusy() && !mtrFreezeRayRight.isBusy()) {
                bHolstered=true;
            } else if((mtrFreezeRayLeft.getCurrentPosition()<10) &&
                (mtrFreezeRayRight.getCurrentPosition()<10)) {
                bHolstered=true;
            
            } else {
                linopMode.telemetry.addData(gstrClassName, "motors busy holstering");
                linopMode.telemetry.update();
            }
        }
    }
    
    
    public void autonShootPixel2(LinearOpMode linopMode,int nHeight,double dBipodLeftPos,
        double dBipodRightPos,double dTriggerPos,long lTargetTime,long lMaxTime){
        long lTimestamp=System.currentTimeMillis();
        long lShotStart;
        boolean bAimed=false, bShot=false, bHolstered=false;
        //setup motors
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayLeft.setTargetPosition(nHeight);
        mtrFreezeRayRight.setTargetPosition(nHeight);
        mtrFreezeRayLeft.setPower(RAY_PWR);
        mtrFreezeRayRight.setPower(RAY_PWR);
        //wait for bar to raise
        while (linopMode.opModeIsActive() && !linopMode.isStopRequested() && !bAimed) {
            if ((System.currentTimeMillis() - lTimestamp) > lMaxTime) {
                linopMode.telemetry.addData(gstrClassName, "timeout");
                linopMode.telemetry.update();
                return;
            }
            if(!mtrFreezeRayLeft.isBusy() && !mtrFreezeRayRight.isBusy()) {
                bAimed=true;
            } else if((mtrFreezeRayLeft.getCurrentPosition()>=(nHeight-10)) &&
                (mtrFreezeRayRight.getCurrentPosition()>=(nHeight-10))) {
                bAimed=true;
            
            } else {
                linopMode.telemetry.addData(gstrClassName, "motors busy aiming");
                linopMode.telemetry.update();
            }
        }
        mtrFreezeRayRight.setPower(0);
        mtrFreezeRayLeft.setPower(0);
        if(!bAimed) return;
        //set servos
        srvoBipodLeft.setPosition(BIPOD_LEFT_NEUTRAL);
        srvoBipodRight.setPosition(BIPOD_RIGHT_NEUTRAL);
        //srvoTrigger.setPosition(TRIGGER_OPEN);
        srvoBipodLeft.setPosition(dBipodLeftPos);
        srvoBipodRight.setPosition(dBipodRightPos);
        srvoTrigger.setPosition(dTriggerPos);
        
        //wait for shot to hit target
        lShotStart=System.currentTimeMillis();
        while (linopMode.opModeIsActive() && !linopMode.isStopRequested() && !bShot) {
            if ((System.currentTimeMillis() - lTimestamp) > lMaxTime) {
                linopMode.telemetry.addData(gstrClassName, "timeout");
                linopMode.telemetry.update();
                return;
            }
            if((System.currentTimeMillis()-lShotStart)>lTargetTime) {
                 bShot=true;
           } else {
                linopMode.telemetry.addData(gstrClassName, "waiting for shot to hit");
                linopMode.telemetry.update();
            }
        }
        
        mtrFreezeRayRight.setPower(0);
        mtrFreezeRayLeft.setPower(0);
        if(!bShot) return;
        //lower FreezeRay
        
        unholsterFreezeRay();
        linopMode.sleep(1200);
        
        
        
        //holster freeze ray
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayLeft.setTargetPosition(0);
        mtrFreezeRayRight.setTargetPosition(0);
        mtrFreezeRayLeft.setPower(RAY_PWR/2);
        mtrFreezeRayRight.setPower(RAY_PWR/2);
        //wait for holstering
        while (linopMode.opModeIsActive() && !linopMode.isStopRequested() && !bHolstered) {
            if ((System.currentTimeMillis() - lTimestamp) > lMaxTime) {
                linopMode.telemetry.addData(gstrClassName, "timeout");
                linopMode.telemetry.update();
                return;
            }
            if(!mtrFreezeRayLeft.isBusy() && !mtrFreezeRayRight.isBusy()) {
                bHolstered=true;
            } else if((mtrFreezeRayLeft.getCurrentPosition()<10) &&
                (mtrFreezeRayRight.getCurrentPosition()<10)) {
                bHolstered=true;
            
            } else {
                linopMode.telemetry.addData(gstrClassName, "motors busy holstering");
                linopMode.telemetry.update();
            }
        }
        srvoTrigger.setPosition(TRIGGER_CLOSE);
    }
     public void autonShootPixel3(LinearOpMode linopMode,double dBipodLeftPos,
        double dBipodRightPos,long lTargetTime,long lMaxTime){
        long lTimestamp=System.currentTimeMillis();
        long lShotStart;
        boolean bAimed=false, bShot=false, bHolstered=false;
        //setup motors
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayLeft.setTargetPosition(RAY_POS_AUTO);
        mtrFreezeRayRight.setTargetPosition(RAY_POS_AUTO);
        mtrFreezeRayLeft.setPower(RAY_PWR);
        mtrFreezeRayRight.setPower(RAY_PWR);
        //wait for bar to raise
        while (linopMode.opModeIsActive() && !linopMode.isStopRequested() && !bAimed) {
            if ((System.currentTimeMillis() - lTimestamp) > lMaxTime) {
                linopMode.telemetry.addData(gstrClassName, "timeout");
                linopMode.telemetry.update();
                return;
            }
            if(!mtrFreezeRayLeft.isBusy() && !mtrFreezeRayRight.isBusy()) {
                bAimed=true;
            } else if((mtrFreezeRayLeft.getCurrentPosition()>=(RAY_POS_AUTO-10)) &&
                (mtrFreezeRayRight.getCurrentPosition()>=(RAY_POS_AUTO-10))) {
                bAimed=true;
            
            } else {
                linopMode.telemetry.addData(gstrClassName, "motors busy aiming");
                linopMode.telemetry.update();
            }
        }
        mtrFreezeRayRight.setPower(0);
        mtrFreezeRayLeft.setPower(0);
        if(!bAimed) return;
        //set servos
       
        srvoBipodLeft.setPosition(BIPOD_LEFT_NEUTRAL);
        srvoBipodRight.setPosition(BIPOD_RIGHT_NEUTRAL);
        //srvoTrigger.setPosition(TRIGGER_OPEN);
        srvoBipodLeft.setPosition(dBipodLeftPos);
        srvoBipodRight.setPosition(dBipodRightPos);
        linopMode.sleep(1200);
        srvoTrigger.setPosition(TRIGGER_OPEN);

        //wait for shot to hit target
        lShotStart=System.currentTimeMillis();
        while (linopMode.opModeIsActive() && !linopMode.isStopRequested() && !bShot) {
            if ((System.currentTimeMillis() - lTimestamp) > lMaxTime) {
                linopMode.telemetry.addData(gstrClassName, "timeout");
                linopMode.telemetry.update();
                return;
            }
            if((System.currentTimeMillis()-lShotStart)>lTargetTime) {
                 bShot=true;
           } else {
                linopMode.telemetry.addData(gstrClassName, "waiting for shot to hit");
                linopMode.telemetry.update();
            }
        }
        
        mtrFreezeRayRight.setPower(0);
        mtrFreezeRayLeft.setPower(0);
        if(!bShot) {
            return;
        }
        //raise FreezeRay slightly so as not to hit pixel
         mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         mtrFreezeRayLeft.setTargetPosition(RAY_POS_AUTO_LIFT);
         mtrFreezeRayRight.setTargetPosition(RAY_POS_AUTO_LIFT);
         mtrFreezeRayLeft.setPower(RAY_PWR/2);
         mtrFreezeRayRight.setPower(RAY_PWR/2);


         // lower FreezeRay
        unholsterFreezeRay();
        srvoTrigger.setPosition(TRIGGER_CLOSE);
        
//        linopMode.sleep(1000);
        
        
        
        //holster freeze ray
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayLeft.setTargetPosition(0);
        mtrFreezeRayRight.setTargetPosition(0);
        mtrFreezeRayLeft.setPower(RAY_PWR/2);
        mtrFreezeRayRight.setPower(RAY_PWR/2);
        //wait for holstering
//        while (linopMode.opModeIsActive() && !linopMode.isStopRequested() && !bHolstered) {
//            if ((System.currentTimeMillis() - lTimestamp) > lMaxTime) {
//                linopMode.telemetry.addData(gstrClassName, "timeout");
//                linopMode.telemetry.update();
//                return;
//            }
//            if(!mtrFreezeRayLeft.isBusy() && !mtrFreezeRayRight.isBusy()) {
//                bHolstered=true;
//            } else if((mtrFreezeRayLeft.getCurrentPosition()<10) &&
//                (mtrFreezeRayRight.getCurrentPosition()<10)) {
//                bHolstered=true;
//
//            } else {
//                linopMode.telemetry.addData(gstrClassName, "motors busy holstering");
//                linopMode.telemetry.update();
//            }
//        }
        srvoTrigger.setPosition(TRIGGER_CLOSE);
    }
    public void autonRaiseWeapon(LinearOpMode linopMode) {
        //setup motors
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayLeft.setTargetPosition(RAY_POS_AUTO);
        mtrFreezeRayRight.setTargetPosition(RAY_POS_AUTO);
        mtrFreezeRayLeft.setPower(RAY_PWR);
        mtrFreezeRayRight.setPower(RAY_PWR);
    }
    public void autonRaiseWeaponHeight(LinearOpMode linopMode, int nPos) {
        //setup motors
//        mtrFreezeRayLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        mtrFreezeRayRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayLeft.setTargetPosition(nPos);
        mtrFreezeRayRight.setTargetPosition(nPos);
        mtrFreezeRayLeft.setPower(RAY_PWR);
        mtrFreezeRayRight.setPower(RAY_PWR);
    }
    public void autonAimWeapon(LinearOpMode linopMode, double dBipodLeftPos,
                               double dBipodRightPos){
        srvoBipodLeft.setPosition(BIPOD_LEFT_NEUTRAL);
        srvoBipodRight.setPosition(BIPOD_RIGHT_NEUTRAL);
        srvoBipodLeft.setPosition(dBipodLeftPos);
        srvoBipodRight.setPosition(dBipodRightPos);
    }
    public void autonShoot(LinearOpMode linopMode){
        srvoTrigger.setPosition(TRIGGER_OPEN);
    }
    public void autonMakeWeaponSafe(LinearOpMode linopMode){
        srvoTrigger.setPosition(TRIGGER_CLOSE);
        srvoBipodLeft.setPosition(BIPOD_LEFT_NEUTRAL);
        srvoBipodRight.setPosition(BIPOD_RIGHT_NEUTRAL);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFreezeRayLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFreezeRayLeft.setTargetPosition(0);
        mtrFreezeRayRight.setTargetPosition(0);
        mtrFreezeRayLeft.setPower(RAY_PWR/2);
        mtrFreezeRayRight.setPower(RAY_PWR/2);
    }

    /*
    private boolean chkElevManual(OpMode opMode) {
        if(gbJetManual&&!(mtrJetLeft.isBusy())&&!(mtrJetRight.isBusy())) {
            gbJetManual=false;
            mtrJetLeft.setPower(0);
            mtrJetRight.setPower(0);
            mtrJetLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            mtrJetRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            
            return true;
        }
        //check if stick is pushed to zero polar
        //if so move both motors up
        if((-opMode.gamepad2.right_stick_y>=.7) &&
            (opMode.gamepad2.right_stick_x>=-.3)&&
            (opMode.gamepad2.right_stick_x<=.3)){
            opMode.telemetry.addData("Jet Manual","JS 0 polar");
            gbJetManual=true;
            gnManualTarget+=JET_POS_INCR;
            mtrJetRight.setTargetPosition(gnManualTarget);
            mtrJetLeft.setTargetPosition(gnManualTarget);
            mtrJetRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtrJetLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtrJetLeft.setPower(JET_PWR);
            mtrJetRight.setPower(JET_PWR);
            return true;
        }
        
        
        //check if stick is pushed to -45 polar
        //if so move left elevator up
        //if((-opMode.gamepad2.right_stick_y>=.5)&&
        //  (opMode.gamepad2.right_stick_x<=-.5)){
        //  opMode.telemetry.addData("Jet Manual","JS -45 polar");
            
        //  mtrJetLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        //  mtrJetRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        //  gbJetAuto=false;
        //  mtrJetLeft.setPower(JET_UP_PWR);
        //  return true;
        //}
        
        //check if stick is pushed to 45 polar
        //if so move right elevator up
        if((-opMode.gamepad2.right_stick_y>=.5)&&
            (opMode.gamepad2.right_stick_x>=5)){
            opMode.telemetry.addData("Jet Manual","JS 45 polar");
            mtrJetLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            mtrJetRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            gbJetAuto=false;
            mtrJetRight.setPower(JET_UP_PWR);
            return true;
        }
        //check if stick is pushed to -135 polar
        //if so move left elevator down
        if((-opMode.gamepad2.right_stick_y<=-.5)&&
            (opMode.gamepad2.right_stick_x<=-.5)){
            opMode.telemetry.addData("Jet Manual","JS -135 polar");
            mtrJetLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            mtrJetRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            gbJetAuto=false;
            mtrJetLeft.setPower(JET_DN_PWR);
            return true;
        }
        //check if stick is pushed to 135 polar
        //if so move right elevator down
        if((-opMode.gamepad2.right_stick_y<=-.7)&&
            (opMode.gamepad2.right_stick_x>=.5)){
            opMode.telemetry.addData("Jet Manual","JS 135 polar");
            mtrJetLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            mtrJetRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            gbJetAuto=false;
            mtrJetRight.setPower(JET_DN_PWR);
            return true;
        }
        
        
        //check if stick is pushed to 180 polar
        if((-opMode.gamepad2.right_stick_y<=-.7)&&
            (opMode.gamepad2.right_stick_x>=-.3)&&
            (opMode.gamepad2.right_stick_x<=.3)){
            opMode.telemetry.addData("Jet Manual","JS 180 polar");
            gbJetManual=true;
            gnManualTarget-=JET_POS_INCR;
            mtrJetRight.setTargetPosition(gnManualTarget);
            mtrJetLeft.setTargetPosition(gnManualTarget);
            mtrJetRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtrJetLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtrJetLeft.setPower(JET_PWR);
            mtrJetRight.setPower(JET_PWR);
            return true;
        }
        
        opMode.telemetry.addData("Jet Manual","JS Nothing");
        return false;
    
    }
    */
    public void shutdown(OpMode opMode) {
        return;
    
        
    }
    
}
