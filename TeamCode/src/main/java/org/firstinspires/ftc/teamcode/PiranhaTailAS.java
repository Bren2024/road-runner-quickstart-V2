package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class PiranhaTailAS {

    // todo: write your code here
    //this class name
    private String gstrClassName=this.getClass().getSimpleName();

    private Servo srvoTail;

    public static double TAIL_BETWEEN_LEGS = .493d;
    public static double TAIL_REST = .48d;
    public static double TAIL_FLICK = .315d; //.5  - very little,  .3 - too much, .07 - too much, .8 - other way //.32

    public static double TAIL_HFLICK = .375d; //.39
    private static double TAIL_WAG = .38d;
    public static int TAIL_INIT_AUTON = 0;
    public static int TAIL_INIT_TELEOP = 1;
    private static long MS_BETWEEN_CMDS= 500;

    private static int TOUT = 1;
    private static int TIN = 0;
    private int nTailState=TIN;

    private long lTimeLastOperation=System.currentTimeMillis();

    public void initialize(OpMode opMode,int nInitMode) {

        opMode.telemetry.addData(gstrClassName, "Initializing...");


        srvoTail = opMode.hardwareMap.get(Servo.class, "srvoTail");
        if(nInitMode==TAIL_INIT_AUTON) {
            srvoTail.setPosition(TAIL_REST);
        } else {
            srvoTail.setPosition(TAIL_BETWEEN_LEGS);

        }
        nTailState=TIN;

        opMode.telemetry.addData(gstrClassName, "    Initialized");

    }

    public void operate(OpMode opMode) {
        opMode.telemetry.addData("PiranhaTail","Tail:%.2f",
                srvoTail.getPosition());

        long lTimestamp=System.currentTimeMillis();
        if((lTimestamp-lTimeLastOperation)<MS_BETWEEN_CMDS) return;
        //if got here, can look at commands
        if(opMode.gamepad2.back) {//
            lTimeLastOperation=lTimestamp;
            if(nTailState==TIN) {
                srvoTail.setPosition(TAIL_WAG);
                nTailState = TOUT;
            } else { //TOUT
                srvoTail.setPosition(TAIL_BETWEEN_LEGS);
                nTailState=TIN;
            }

            return;
        }




    }

    public void autonFlickPixel(LinearOpMode linopMode,long lSpitMSec,long lDroolMSec) {


        srvoTail.setPosition(TAIL_FLICK);

        linopMode.sleep(lSpitMSec);

        srvoTail.setPosition(TAIL_BETWEEN_LEGS);


    }

    public void autonSetFlickPixel(LinearOpMode linopMode, double Position) {

        srvoTail.setPosition(Position);
    }


    public void shutdown(OpMode opMode) {
        return;
    }

}
