/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */


public class Goggles2V3AS {
    //return values
    public static final int PROP_LEFT = 1;
    public static final int PROP_MID = 2;
    public static final int PROP_RIGHT= 3;
    public static final int PROP_NONE= -1;
    
    public static final int AT_NONE = -9999;
    public static final int AT_NOT_INITIALIZED = -9998;
  
   
    //configuration variables
    public static final int RED_CAM = 1;
    public static final int BLUE_CAM = 2;
    public static final int  TFOD_ACTIVE= 1;
    public static final int  APRILTAG_ACTIVE= 2;

    
    //set below to return correct prop position
    private static final double CUTOFF_LEFT_RED=(54d+326d)/2d;
    private static final double CUTOFF_RIGHT_RED=(326d+589)/2d;
    private static final double CUTOFF_LEFT_BLUE=(54d+324d)/2d;
    private static final double CUTOFF_RIGHT_BLUE=(324d+586)/2d;
    
    private WebcamName webcamnameTfod, webcamnameApril;

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    //private static final String TFOD_MODEL_ASSET = "23905_Red_Prop_v2.tflite";
    String strTFOD_ASSET;
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/23905_Red_Prop_v2.tflite";
    String strTFOD_FILE;
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
       "Prop",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionportal=null;
 
    private String gstrClassName=this.getClass().getSimpleName();
    private int gnTFODCam;//=RED_CAM;
    private int gnActiveCamera=TFOD_ACTIVE;
    
    private boolean gbAprilTagCaptured=false;
    private int gnATId=-9999;
    private double gdATX= -9999.99;
    private double gdATY = -9999.9;
    private double gdATBearing = -9999.9;
    private double gdATRange=0d,gdATHeading=0d,gdATYaw=0d;
    

    /**
     * Initialize the object
     * @param opMode
     * @param nTFODCam
     */
    public void initialize(OpMode opMode,int nTFODCam) {

        opMode.telemetry.addData(gstrClassName,"Initializing");
        //Important note:  TFOD and April are both vision processors
        //                 the vision portal can switch between these processors
        //                 
        
        gnTFODCam=nTFODCam;
        
        
        // Create the TensorFlow processor by using a builder.
        if(nTFODCam==RED_CAM) {
            strTFOD_ASSET = "23905_Red_Prop_v2.tflite";
            strTFOD_FILE = "/sdcard/FIRST/tflitemodels/23905_Red_Prop_v2.tflite";
        } else  {
            strTFOD_ASSET = "23905_Blue_Prop_v2.tflite";
            strTFOD_FILE = "/sdcard/FIRST/tflitemodels/23905_Blue_Prop_v2.tflite";
        }
        tfod = new TfodProcessor.Builder()

            // With the following lines commented out, the default TfodProcessor Builder
            // will load the default model for the season. To define a custom model to load, 
            // choose one of the following:
            //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
            //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
            //.setModelAssetName(TFOD_MODEL_ASSET)
            //.setModelFileName(TFOD_MODEL_FILE)
            //.setModelAssetName(strTFOD_ASSET) do not uncomment
            
            
            .setModelFileName( strTFOD_FILE)

            // The following default settings are available to un-comment and edit as needed to 
            // set parameters for custom models.
            .setModelLabels(LABELS)
            //.setIsModelTensorFlow2(true)
            //.setIsModelQuantized(true)
            //.setModelInputSize(300)
            //.setModelAspectRatio(16.0 / 9.0)

            .build();


        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        
        //now define the cameras
        webcamnameApril=opMode.hardwareMap.get(WebcamName.class, "Webcam April");
        if(nTFODCam==RED_CAM) {
            webcamnameTfod=opMode.hardwareMap.get(WebcamName.class, "Webcam 572f");
        } else {
            webcamnameTfod=opMode.hardwareMap.get(WebcamName.class, "Webcam 472f");
        }
        CameraName switchableCamera = ClassFactory.getInstance()
                    .getCameraManager().nameForSwitchableCamera(webcamnameTfod,webcamnameApril);
          
        // Create the vision portal by using a builder.
        visionportal= new VisionPortal.Builder()
            .setCamera(switchableCamera)
            
            .addProcessors(tfod, aprilTag)
            .build();
       

        //setManualExposure(linopMode,6, 25);  // Use low exposure time to reduce motion blur OG=6,250
        
        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        //vpbuilder.addProcessors(tfod,aprilTag);

        // Build the Vision Portal, using the above settings.
        //gvisionportal = vpbuilder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);




        // Disable or re-enable the TFOD processor at any time.
        //gvisionportal.setActiveCamera(webcamnameTfod);
        visionportal.setProcessorEnabled(tfod, true);
        visionportal.setProcessorEnabled(aprilTag,false);
     
        gnActiveCamera=TFOD_ACTIVE;
        
    }
    
    public int getActiveCamera() {
        return gnActiveCamera;
    }
    public void operate(OpMode opMode,boolean bDisplay) {

        if(gnActiveCamera==TFOD_ACTIVE){
            if(bDisplay)telemetryTfod(opMode);
        } else {
            if(bDisplay) telemetryAprilTag(opMode);
        }

         
         

    }   // end runOpMode()

    

    /**
     * Add telemetry for TensorFlow Object Detection (TFOD) recognitions.
     */
    public void telemetryTfod(OpMode opMode) {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        opMode.telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            opMode.telemetry.addData(""," ");
            opMode.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            opMode.telemetry.addData("- Position", "x=%.0f y=%.0f", x, y);
            opMode.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        opMode.telemetry.update();
    }   // end method telemetryTfod()

    /**
     * Add telemetry for AprilTag recognitions.
     * @param opMode
     */
    public void telemetryAprilTag(OpMode opMode) {
        boolean bFound=false;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        opMode.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            bFound=true;
            if (detection.metadata != null) {
                opMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                opMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                opMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                opMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                opMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                opMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        if(bFound) {
            // Add "key" information to telemetry
            
            opMode.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            opMode.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            opMode.telemetry.addLine("RBE = Range, Bearing & Elevation");
            
        }
        opMode.telemetry.update();    
    }   // end method telemetryAprilTag()
/*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void  setManualExposure(LinearOpMode linopMode, int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionportal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionportal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            linopMode.telemetry.addData("Camera", "Waiting");
            linopMode.telemetry.update();
            while (!linopMode.isStopRequested() && (visionportal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                linopMode.sleep(20);
            }
            linopMode.telemetry.addData("Camera", "Ready");
            linopMode.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!linopMode.isStopRequested())
        {
            ExposureControl exposureControl = visionportal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                linopMode.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            linopMode.sleep(20);
            GainControl gainControl = visionportal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            linopMode.sleep(20);
        }
    }
    /**
     * find position of prop
     * @param linopMode
     * @param lTimeout
     * @param nTFODCam  RED_CAM or BLUE_CAM
     * @return PROP_NONE, PPRP_LEFT, PROP_MID, PROP_RIGHT
     *
     */
    public int findProp(LinearOpMode linopMode, long lTimeout) {
        
        long lTimeStamp=System.currentTimeMillis();
        double dXPos=-1d, dConfidence=0;
        
        while (((System.currentTimeMillis()-lTimeStamp)< lTimeout) 
             && linopMode.opModeIsActive()) {
                 
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            linopMode.telemetry.addData("# Objects Detected", currentRecognitions.size());
    
            // Step through the list of recognitions and display info for each one.
            dConfidence=0;
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                double conf =  recognition.getConfidence() * 100d;
                linopMode.telemetry.addData(""," ");
                linopMode.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                linopMode.telemetry.addData("- Position", "x=%.0f y=%.0f", x, y);
                linopMode.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                
                if((conf>dConfidence)) {
                    dXPos=x;
                    dConfidence=conf;
                }
            }   // end for() loop
             linopMode.telemetry.update();
            // Share the CPU.
            linopMode.sleep(20);
            
            if(currentRecognitions.size()>0) {
                linopMode.telemetry.addData("Found prop!!","Pos=%.2f",dXPos);
                linopMode.telemetry.update();
            
                //shutdown(linopMode);
                if(gnTFODCam==RED_CAM) {
                    if(dXPos<CUTOFF_LEFT_RED) return PROP_LEFT;
                    if(dXPos>CUTOFF_RIGHT_RED) return PROP_RIGHT;
                } else {//using BLUE_CAM
                    if(dXPos<CUTOFF_LEFT_BLUE) return PROP_LEFT;
                    if(dXPos>CUTOFF_RIGHT_BLUE) return PROP_RIGHT;
                }
                return PROP_MID;
            }
            
            
        }//end while loop
        linopMode.telemetry.addData(gstrClassName,"NoProp");
        linopMode.telemetry.update();
        //shutdown(linopMode);
        return PROP_NONE;   
    }


    /**
     * switch camera to AprilTag Camera
     * @param opMode
     */
    public void switchCameraToAprilTag() {
        visionportal.setActiveCamera(webcamnameApril);
        visionportal.setProcessorEnabled(tfod, false);
        visionportal.setProcessorEnabled(aprilTag,true);
        tfod.shutdown();

        gnActiveCamera=APRILTAG_ACTIVE;

    }

    /** Find april tag
     *
     * @param linearOpMode
     * @param lTimeOut
     * @return AprilTag ID or -1 if not found after timeout
     */
    public int findAprilTag(LinearOpMode linearOpMode, int nRequestedTagID,long lTimeOut) {
        boolean bTargetFound     = false;
        long lTimeStamp=System.currentTimeMillis();
        while (linearOpMode.opModeIsActive()&&((System.currentTimeMillis()-lTimeStamp)<lTimeOut)) {
            bTargetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == nRequestedTagID) {
                        // Yes, we want to use this tag.
                        bTargetFound = true;
                        desiredTag = detection;
                        linearOpMode.telemetry.addData(gstrClassName, "Tag ID %d found", detection.id);
                        //break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        linearOpMode.telemetry.addData(gstrClassName, "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    linearOpMode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
            linearOpMode.telemetry.update();

            //
            if (bTargetFound) {
                linearOpMode.telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                linearOpMode.telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                linearOpMode.telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                linearOpMode.telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
                linearOpMode.telemetry.addData("x,y","x=%.2f y=%.2f", desiredTag.ftcPose.x,desiredTag.ftcPose.y);
                
                linearOpMode.telemetry.update();
                
                gdATRange      = (desiredTag.ftcPose.range);
                gdATHeading    = desiredTag.ftcPose.bearing;
                gdATYaw        = desiredTag.ftcPose.yaw;
                gdATX=desiredTag.ftcPose.x;
                gdATY=desiredTag.ftcPose.y;
                gdATBearing=desiredTag.ftcPose.bearing;
                return desiredTag.id;
            }
            else {
                linearOpMode.telemetry.addData(gstrClassName,"No April Tag found\n");
                linearOpMode.telemetry.update();
            }
            linearOpMode.sleep(10);
        }
        //timeout no tag found
        return -1;
    }


     public float getAprilTagX() {
        return (float)gdATX;
    }
    
    public float getAprilTagY() {
        return (float)gdATY;
    }
    public float getAprilTagYaw() {
        return (float)gdATYaw;
    }
    
 /**
     * Shut down vision
      * @param opMode
     */
    public void shutdown(OpMode opMode) {
        opMode.telemetry.addData(gstrClassName,"Shutting Down");
        //if(visionportal.getCameraState()== CameraState.STREAMING) {
        //visionportal.stopStreaming();
        //}
        //if(visionportal.getProcessorEnabled(tfod))
        //visionportal.setProcessorEnabled(tfod,false);
        //if(visionportal.getProcessorEnabled(aprilTag))
        //visionportal.setProcessorEnabled(aprilTag,false);

        // Save more CPU resources when camera is no longer needed.

        visionportal.close();
        tfod.shutdown();
        //no shutdown for apriltag processor
        
    }
    

}   // end class
