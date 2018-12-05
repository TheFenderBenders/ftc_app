/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="LM3_Autonomous", group="Linear Opmode")
public class LM3_Autonomous extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AW8pxAb/////AAABmc2pCnd0uUidheyLY5krCRBcvgnlBqrElE/ZP/pTLqZoxVQ8COgVDVpCp0pOmtF6HP9kyk7kh9Qjq0A6ND0F7A0iemGwWN2RxixFEOSWiDrSbc46XnYYpF+qCAkHx2w2e4tvJD4REtBPVTd7URXPnMEKqJde9cWVQc6D9gOFAa42CnkYsuvJZ2Kn2Lc51kuqyJ0szGwPjZUsA5vZ1vENH75y2tuym8jY4oRl2BYsmehEotnxApXt/6D+gdYsb7cGAyZuHxXx00zp+gGTlnrhJdEx9DnQVjI2HLBi6j848ayI200c8jCVqiVtv+NExtP3NCDY66YGGKp+so0pJ7MRUWYrJ7+4n4kGKg59erl+UIjO";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */

    private TFObjectDetector tfod;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor liftDrive = null;

    final int HANGING = 1;
    final int UNLATCH = 2;
    final int BACKUP = 3;
    final int SAMPLE = 4;
    final int PUSH_AND_CLAIM = 5;
    final int DONE = 100;
    final int TEST = 101;

    final int FINALSAMPLE = 12;
    final int turn = 14;
    boolean inCrater = false;
    boolean turned = false;
    boolean backed = false;
    int state;
    boolean foundGold = false;
    boolean timed = false;
    long TimeSample;
    int positionOfGold;// 0 = Gold is Left      1 = Gold is in the middle     2 = Gold is right
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        liftDrive = hardwareMap.get(DcMotor.class, "lift_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        liftDrive.setDirection(DcMotor.Direction.FORWARD);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addLine("Sorry! This device is not compatible with TFOD");
        }

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // start off hanging
        state = UNLATCH;

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        boolean tfodInitialized = false;
        long tStart = System.currentTimeMillis();

        while (opModeIsActive()) {
            if (tfod != null) {
                switch (state) {
                    case TEST:
                        telemetry.addLine("in TEST");
                        telemetry.update();
                        if (System.currentTimeMillis() - tStart < 1000) {
                            leftDrive.setPower(0.3);
                            rightDrive.setPower(0.3);
                        }
                        else {
                            leftDrive.setPower(0.0);
                            rightDrive.setPower(0.0);
                            state = DONE;
                        }
                        break;

                        case HANGING:
                        if (System.currentTimeMillis() - tStart < 8700) {
                            liftDrive.setPower(1.0);
                        } else {
                            liftDrive.setPower(0.0);
                            state = UNLATCH;
                            tStart = System.currentTimeMillis();
                        }
                        break;

                        case UNLATCH:
                        if (System.currentTimeMillis() - tStart < 500) { // was 350
                            leftDrive.setPower(-0.5);
                            rightDrive.setPower(0.5);
                        } else {
                            leftDrive.setPower(0.0);
                            rightDrive.setPower(0.0);
                            Thread.sleep(500);
                            state = BACKUP;
                            tStart = System.currentTimeMillis();
                        }
                        break;

                        case BACKUP:
                        if (System.currentTimeMillis() - tStart < 150) {
                            leftDrive.setPower(-0.4);
                            rightDrive.setPower(-0.4);
                        } else {
                            leftDrive.setPower(0.0);
                            rightDrive.setPower(0.0);
                            state = SAMPLE;
//                            telemetry.addLine("sleeping for 5 seconds");
//                            telemetry.update();
//                            Thread.sleep(5000);
                            tStart = System.currentTimeMillis();
                            TimeSample = tStart;
                        }
                        break;

                        case SAMPLE:
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            if (updatedRecognitions.size() > 0) {
                                telemetry.addData("# Objects Detected", updatedRecognitions.size());
                                telemetry.update();
                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        foundGold = true;
                                        
                                        // gold found; turn power off right away
                                        leftPower = 0.0;
                                        rightPower = 0.0;
                                        leftDrive.setPower(leftPower);
                                        rightDrive.setPower(rightPower);
                                        tStart = System.currentTimeMillis();

                                        telemetry.addData("Found Gold at ", recognition.getLeft());
                                        telemetry.update();

                                        long pos = System.currentTimeMillis()-TimeSample;
                                        
                                        if(pos<900){
                                            telemetry.addLine("Gold | Silver | Silver");
                                            positionOfGold = 0;
                                            state = PUSH_AND_CLAIM;
                                        }
                                        else if((pos>1400)&&(pos<2400)){
                                            telemetry.addLine("Silver | Gold | Silver");

                                            positionOfGold = 1;
                                            state = PUSH_AND_CLAIM;
                                        }
                                        else if(pos>3000){
                                            telemetry.addLine("Silver | Silver | Gold");
                                            positionOfGold = 2;
                                            state = PUSH_AND_CLAIM;
                                        }
                                        tStart = System.currentTimeMillis();
                                    }
                                }
                            }

                            if (!foundGold) { // gold not found. Turn to the right
                                leftPower = 0.4;
                                rightPower = -0.4;
                                leftDrive.setPower(leftPower);
                                rightDrive.setPower(rightPower);
                            } 
/*                            else {
                                leftPower = 0.0;
                                rightPower = 0.0;
                                leftDrive.setPower(leftPower);
                                rightDrive.setPower(rightPower);
                                tStart = System.currentTimeMillis();
                            }
*/
                            Thread.sleep(100);
                            telemetry.update();
                        }
                        break;

                        case PUSH_AND_CLAIM:
                        if(positionOfGold == 0){
                            state = DONE;
                            //left
                        }
                        else if(positionOfGold == 1){
                            //middle. Go fwd to depot
                            if (System.currentTimeMillis() - tStart < 2000) {
                                leftPower = -0.5;
                                rightPower = -0.5;
                                leftDrive.setPower(leftPower);
                                rightDrive.setPower(rightPower);
                            }
                            else {
                                state = DONE;
                            }

                        }
                        else if (positionOfGold == 2){
                            //right
                            state = DONE;
                        }
                        break;

                        case turn:
                        if(System.currentTimeMillis()-tStart<200){
                            leftDrive.setPower(-0.4);
                            rightDrive.setPower(0.4);
                        }
                        else{
                            leftDrive.setPower(0.0);
                            rightDrive.setPower(0.0);
                            state = FINALSAMPLE;
                        }
                        break;

                        case FINALSAMPLE:
                        tStart = System.currentTimeMillis();
                        if((System.currentTimeMillis()-tStart<2000)&&(!backed)){
                            leftDrive.setPower(0.3);
                            rightDrive.setPower(0.3);
                        }
                        else{


                            leftDrive.setPower(0.0);
                            rightDrive.setPower(0.0);
                            backed = true;
                            state = DONE;

                        }
                        break;

                        case DONE:
                            leftDrive.setPower(0.0);
                            rightDrive.setPower(0.0);
                            break;
                    }
        }
    }

    if(tfod !=null)
    {
        tfod.shutdown();
    }
}

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


}





/*


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

                    // this is where our TF code goes
                    telemetry.addData("In sample statement", 10);

                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object(s) Detected", updatedRecognitions.size());

                        for (Recognition recognition : updatedRecognitions) {
                            if ((recognition.getLabel().equals(LABEL_GOLD_MINERAL))) {
                                telemetry.addData("Gold Mineral Detected", 10);
                                goldMineralXL = (int) recognition.getLeft();

                                // go straight back if robot sees gold mineral
//                                tStart = System.currentTimeMillis();

//                                while (System.currentTimeMillis() - tStart < 400) {
//                                    leftDrive.setPower(-1.0);
//                                    rightDrive.setPower(-1.0)
//                               }

                            }


                        }
                    }
                    else {
                        telemetry.addData("NO OBJECTS DETECTED!", 10);
                    }
                    break;

                    case REVERSED:
                        if (System.currentTimeMillis() - tStart < 1500) {
                            leftDrive.setPower(-0.2);
                            rightDrive.setPower(-0.2);
                        } else {
                            leftDrive.setPower(0.0);
                            rightDrive.setPower(0.0);
                            state = TURNED;
                            tStart = System.currentTimeMillis();
                        }
                        break;

                    case TURNED:
                        if (System.currentTimeMillis() - tStart < 600) {
                            leftDrive.setPower(0.5);
                            rightDrive.setPower(-0.5);
                        } else {
                            leftDrive.setPower(0.0);
                            rightDrive.setPower(0.0);
                            state = STRAIGHTENED;
                            tStart = System.currentTimeMillis();
                        }
                        break;

                    case STRAIGHTENED:
                        if (System.currentTimeMillis() - tStart < 850) {
                            leftDrive.setPower(-1.0);
                            rightDrive.setPower(-1.0);
                        } else {
                            leftDrive.setPower(0.0);
                            rightDrive.setPower(0.0);
                            state = INDEPOT;
                            tStart = System.currentTimeMillis();
                        }
                        break;

                    case INDEPOT:
                        if (System.currentTimeMillis() - tStart < 250) {
                            leftDrive.setPower(1.0);
                            rightDrive.setPower(1.0);
                        } else {
                            leftDrive.setPower(0.0);
                            rightDrive.setPower(0.0);
                            state = CLAIMED;
                            tStart = System.currentTimeMillis();
                        }
                        break;
                    case CLAIMED:
                        if (System.currentTimeMillis() - tStart < 1000) {
                            leftDrive.setPower(-0.5);
                            rightDrive.setPower(0.5);
                        } else {
                            leftDrive.setPower(0.0);
                            rightDrive.setPower(0.0);
                            state = TOWARDCRATER;
                            tStart = System.currentTimeMillis();
                        }
                        break;
                    case TOWARDCRATER:
                        if (System.currentTimeMillis() - tStart < 2000) {
                            leftDrive.setPower(1.0);
                            rightDrive.setPower(1.0);
                        } else {
                            leftDrive.setPower(0.0);
                            rightDrive.setPower(0.0);
                            inCrater = true;
                        }
                        break;

                    case DONE:
                        break;
                    }

                    idle();
                    // Show the elapsed game time and wheel power.
                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("Motors", "left (%.2f), right (%.2f)", 0.5, 0.5);
                    if (inCrater) {
                        telemetry.addData("Status", "In Crater");
                    }
                    telemetry.update();
        }
  */
