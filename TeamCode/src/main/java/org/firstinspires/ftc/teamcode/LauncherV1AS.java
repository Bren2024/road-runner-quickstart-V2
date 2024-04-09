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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

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

public class LauncherV1AS {
    
    //this class name
    private String gstrClassName=this.getClass().getSimpleName();
    
    //Declare OpMode members
    private Servo srvoVector;


    //servo positions
    private static double STORE_PIN = .5d;
    private static double PULL_PIN = .47d;

    public void initialize(OpMode opMode) {
  
        opMode.telemetry.addData(gstrClassName, "Initializing...");

        //Vector Servo
        srvoVector = opMode.hardwareMap.get(Servo.class, "srvoVector");
        srvoVector.setPosition(STORE_PIN);
        
        opMode.telemetry.addData(gstrClassName, "    Initialized");
        

    }

    public void operate(OpMode opMode) {
        opMode.telemetry.addData(gstrClassName,"Launcher Pos:%.2f",
           srvoVector.getPosition());

        if(opMode.gamepad2.guide) {
            srvoVector.setPosition(PULL_PIN);
        }
    }
    public void shutdown(OpMode opMode) {
        return;
    }

}
