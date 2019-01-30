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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

@Autonomous(name="SQT Autonomous Crater Test", group="Linear Opmode")
public class SQT_Autonomous_Crater_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor liftDrive = null;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AW8pxAb/////AAABmc2pCnd0uUidheyLY5krCRBcvgnlBqrElE/ZP/pTLqZoxVQ8COgVDVpCp0pOmtF6HP9kyk7kh9Qjq0A6ND0F7A0iemGwWN2RxixFEOSWiDrSbc46XnYYpF+qCAkHx2w2e4tvJD4REtBPVTd7URXPnMEKqJde9cWVQc6D9gOFAa42CnkYsuvJZ2Kn2Lc51kuqyJ0szGwPjZUsA5vZ1vENH75y2tuym8jY4oRl2BYsmehEotnxApXt/6D+gdYsb7cGAyZuHxXx00zp+gGTlnrhJdEx9DnQVjI2HLBi6j848ayI200c8jCVqiVtv+NExtP3NCDY66YGGKp+so0pJ7MRUWYrJ7+4n4kGKg59erl+UIjO";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    boolean encoderTest_defineMotorTypeOnce = false;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    boolean aButton, bButton, touched;
    boolean depot;

    private int goldPos = -1; // position of gold mineral - 0 means G-S-S, 1 means S-G-S, and 2 means S-S-G

    //all tasks
    final int SAMPLE_RECOGNITION = 0;
    final int LANDING_AND_UNLATCHING_FROM_LANDER = 1;
    final int BACKUP = 2;
    final int GOLD_CASE_ZERO = 3;//when gold is on the left side
    final int GOLD_CASE_ONE = 4; //when gold is in the middle
    final int GOLD_CASE_TWO = 5;// when gold os on the right side
    final int GOTO_CRATER_ZERO = 6;
    final int GOTO_CRATER_ONE = 7;
    final int GOTO_CRATER_TWO = 8;
    final int GO_STRAIGHT_GOLD_TWO = 9;
    final int RETRACT_LIFT = 10;
    final int BACKOUTZEROANDONE = 11;
    final int BACKOUTTWO = 12;
    final int ALLTASKSCOMPLETED = 100;
    final int ENCODERTEST = 101;
    final int RETRACT_LIFT_TIME = 10000;

    final int CRATER_SIDE = 13;
    final int DEPOT_SIDE = 14;
    final int CASE_ONE = 15;
    final int CASE_ONE_GO_STRAIGHT = 16;

/*

 */
    long tStart;
    int currentTask;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    static final int CUTTOFFPOINT = 500;

    @Override
    public void runOpMode() {

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addLine("Sorry! This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        } else {
            telemetry.addLine("Sorry! Could not activate TFOD");
        }

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        liftDrive = hardwareMap.get(DcMotor.class, "lift_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        liftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Calibrating ", "IMU");
        telemetry.update();

        // set up IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // compensate for vertical Rev hub
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

        sleep(100); //Changing modes requires a delay before doing anything else

        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);

        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);

        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        sleep(100); //Changing modes again requires a delay

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        if (tfod != null) {
            List<Recognition> updatedRecognitionsInit = tfod.getUpdatedRecognitions();
            if (updatedRecognitionsInit != null) {
                int initSize = updatedRecognitionsInit.size();
                if (initSize > 0) {
                    telemetry.addData("# Objects Detected", updatedRecognitionsInit.size());
                    telemetry.update();
                    if(initSize == 2){
                        depot = true;
                    }
                    else if(initSize>2){
                        depot = false;
                    }
                    else{
                        telemetry.addLine("ERROR: CANNOT TELL WHETHER DEPOT OR CRATER. CAUSE: NOT SEEING ANY OBJECTS(INSIDE FOUND OBJECTS)");
                        telemetry.update();
                    }
                }
                else{
                    telemetry.addLine("ERROR: CANNOT TELL WHETHER DEPOT OR CRATER. CAUSE: NOT SEEING ANY OBJECTS(OUTSIDE FOUND OBJECTS)");
                    telemetry.update();
                }

            }
        }



        waitForStart();
        runtime.reset();
        currentTask = SAMPLE_RECOGNITION;
        tStart = System.currentTimeMillis();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double leftPower = 0.0;
            double rightPower = 0.0;


            switch (currentTask){
                case SAMPLE_RECOGNITION:
                    goldPos = findGoldPosition();
                    if(depot){
                        currentTask = DEPOT_SIDE;
                    }
                    else{
                        currentTask = CRATER_SIDE;
                    }
                    break;



                case DEPOT_SIDE:
                    if(goldPos == 1){
                        currentTask = CASE_ONE;
                    }
                    break;



                case CRATER_SIDE:
                    if(goldPos == 1){
                        currentTask = CASE_ONE;
                    }
                    break;

                case CASE_ONE:
                    rotate(-20, 0.4);
                    currentTask = CASE_ONE_GO_STRAIGHT;
                    break;

                case CASE_ONE_GO_STRAIGHT:
                    break;
/*
crater
if (System.currentTimeMillis() - tStart < 3000) {
                        leftDrive.setPower(-0.5);
                        rightDrive.setPower(-0.5);
                    }
                    after this the sample case completes

 depot
  if (System.currentTimeMillis() - tStart < 3000) {
                        leftDrive.setPower(-0.5);
                        rightDrive.setPower(-0.5);
                    }
                    after this claim

 */





            }










//OLD CODE down NEW CODE ^

            switch (currentTask) {
                case SAMPLE_RECOGNITION:
                    goldPos = findGoldPosition();
                    if (goldPos != -1) {
//                        currentTask = LANDING_AND_UNLATCHING_FROM_LANDER;
                        currentTask = LANDING_AND_UNLATCHING_FROM_LANDER;

                        tStart = System.currentTimeMillis();
                    }
                    else if (System.currentTimeMillis() - tStart > 6000) { // gold not found for 6 seconds...move on and go straight to the depot
                        currentTask = LANDING_AND_UNLATCHING_FROM_LANDER;
                        goldPos = 1;
                        tStart = System.currentTimeMillis();
                    }
                break;

                case LANDING_AND_UNLATCHING_FROM_LANDER:
                    if (System.currentTimeMillis() - tStart < RETRACT_LIFT_TIME) {
                        liftDrive.setPower(1.0);
                    } else {
                        tStart = System.currentTimeMillis();
                        liftDrive.setPower(0.0);

                        // unlatch
                        rotate(20, 0.4);
                        tStart = System.currentTimeMillis();
                        currentTask = BACKUP;
                    }
                    break;

                case BACKUP:
                    if (System.currentTimeMillis() - tStart < 250) {
                        leftDrive.setPower(-0.5);
                        rightDrive.setPower(-0.5);
                    }
                    else {
                        leftDrive.setPower(0.0);
                        rightDrive.setPower(0.0);
                        tStart = System.currentTimeMillis();
                        switch (goldPos) {
                            case 0:
                                currentTask = GOLD_CASE_ZERO;
                                break;
                            case 1:
                                currentTask = GOLD_CASE_ONE;
                                break;
                            case 2:
                                currentTask = GOLD_CASE_TWO;
                                break;
                        }
                    }
                    break;

                case GOLD_CASE_ZERO:
                    if (System.currentTimeMillis() - tStart < 1750) {
                        // don't turn. Just go straight towards gold
                        leftDrive.setPower(-0.5);
                        rightDrive.setPower(-0.5);
                    }
                    else {
                        leftDrive.setPower(0.0);
                        rightDrive.setPower(0.0);
                        currentTask = RETRACT_LIFT;
                        tStart = System.currentTimeMillis();
                    }
                    break;

                case GOLD_CASE_ONE:
                    rotate(-20, 0.4);
                    currentTask = GOTO_CRATER_ONE;
                    tStart = System.currentTimeMillis();
                    break;

                case GOLD_CASE_TWO:
                    rotate(-55, 0.4);
                    currentTask = GO_STRAIGHT_GOLD_TWO;
                    tStart = System.currentTimeMillis();
                    break;

                case GOTO_CRATER_ONE:
                    if (System.currentTimeMillis() - tStart < 3000) {
                        leftDrive.setPower(-0.5);
                        rightDrive.setPower(-0.5);
                    }
                    else {
                        leftDrive.setPower(-0.0);
                        rightDrive.setPower(-0.0);
                        currentTask = RETRACT_LIFT;
                        tStart = System.currentTimeMillis();
                    }
                    break;

                case GO_STRAIGHT_GOLD_TWO:
                    if (System.currentTimeMillis() - tStart < 2500) {
                        // don't turn. Just go straight towards gold
                        leftDrive.setPower(-0.5);
                        rightDrive.setPower(-0.5);
                    }
                    else {
                        leftDrive.setPower(0.0);
                        rightDrive.setPower(0.0);
                        currentTask = RETRACT_LIFT;
                        tStart = System.currentTimeMillis();
                    }
                    break;

                case RETRACT_LIFT:
                    if (System.currentTimeMillis() - tStart < 800) {
                        liftDrive.setPower(-1.0);
                    }
                    else {
                        liftDrive.setPower(0.0);
                       switch (goldPos){
                           case 0:
                              currentTask = BACKOUTZEROANDONE;
                               tStart = System.currentTimeMillis();
                               break;
                           case 1:
                               currentTask = BACKOUTZEROANDONE;
                               tStart = System.currentTimeMillis();
                               break;
                           case 2:
                               break;

                       }

                    }
                    break;

                case BACKOUTZEROANDONE:
                    if(System.currentTimeMillis()-tStart<2000) {
                        leftDrive.setPower(1.0);
                        rightDrive.setPower(1.0);
                    }
                    else{
                        leftDrive.setPower(0.0);
                        rightDrive.setPower(0.0);
                    currentTask = ALLTASKSCOMPLETED;
                    }
                        break;

                case ALLTASKSCOMPLETED:
                    leftDrive.setPower(0.0);
                    rightDrive.setPower(0.0);
                    telemetry.addData("Goldpos=", goldPos);
                    telemetry.update();
                    break;

                case ENCODERTEST:
                    if(!encoderTest_defineMotorTypeOnce) {
                        GoStraight(24);
                        encoderTest_defineMotorTypeOnce = true;
                    }

                        telemetry.addLine("done");


                    break;


            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Gold location: ", goldPos);
            telemetry.update();
            idle();
        }
    }

    // find gold position using TFLite
    private int findGoldPosition() {
        int goldMineralX, silverMineral1X, silverMineral2X, gPos = -1;

        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() > 0) {
                    goldMineralX = -1;
                    silverMineral1X = -1;
                    silverMineral2X = -1;

                    telemetry.addData("# Objects Detected", updatedRecognitions.size());
                    telemetry.update();

                    for (Recognition recognition : updatedRecognitions) {
                        int left = (int) recognition.getLeft();
                        int top = (int) recognition.getTop();
                        telemetry.addData("top =  ", top);

//                        if (top > CUTTOFFPOINT) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
  //                      }
                    }
                    if (silverMineral2X == -1) {
                        if (goldMineralX > silverMineral1X) {
                            gPos = 1;
                            telemetry.addLine("S-G-S");
                            telemetry.update();
                            currentTask = LANDING_AND_UNLATCHING_FROM_LANDER;
                        } else {
                            gPos = 0;
                            telemetry.addLine("G-S-S");
                            telemetry.update();
                            currentTask = LANDING_AND_UNLATCHING_FROM_LANDER;
                        }
                    } else {
                        gPos = 2;
                        telemetry.addLine("S-S-G");
                        telemetry.update();
                        currentTask = LANDING_AND_UNLATCHING_FROM_LANDER;
                    }
                }
            }

        }
        return gPos;
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

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {

        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
            }

        // turn the motors off.
        rightDrive.setPower(0);
        leftDrive.setPower(0);

        // wait for rotation to stop.
        sleep(100);

        // reset angle tracking on new heading.
        resetAngle();
    }

    void GoStraight(int distanceInInches) {

        // set right motor to run without regard to an encoder.
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int newTarget = leftDrive.getCurrentPosition() - (int)(distanceInInches * COUNTS_PER_INCH);

        leftDrive.setTargetPosition(newTarget);
        // set left motor to run to target encoder position and stop with brakes on.
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set both motors to 25% power. Movement will start.
        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);

        while (leftDrive.isBusy()) {
            telemetry.addData("newTarget=", newTarget);
            telemetry.addData("curPos=", leftDrive.getCurrentPosition());
            telemetry.addData("encoder-fwd", leftDrive.getCurrentPosition() + "  busy=" + leftDrive.isBusy());
            telemetry.update();
            idle();
        }

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}
