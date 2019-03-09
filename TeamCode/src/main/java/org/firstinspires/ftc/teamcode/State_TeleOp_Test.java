/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="State Teleop Test", group="Linear Opmode")
public  class State_TeleOp_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor armDrive = null;
    private DcMotor linearDrive = null;
    private DigitalChannel digitalTouch = null;
    private CRServo collectorDrive = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private CRServo liftDrive = null;
    long tStart;
    static final int COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 25.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION ;
    boolean firstUse = true;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        armDrive = hardwareMap.get(DcMotor.class, "arm_drive");
        armDrive.setDirection(DcMotor.Direction.REVERSE);
//        armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearDrive = hardwareMap.get(DcMotor.class, "linear_drive");
        linearDrive.setDirection(DcMotor.Direction.FORWARD);
        linearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collectorDrive = hardwareMap.get(CRServo.class, "collector_drive");
//        digitalTouch = hardwareMap.get(DigitalChannel.class, "limit_switch");
//        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        liftDrive = hardwareMap.get(CRServo.class,"lift_drive");
        liftDrive.setDirection(DcMotorSimple.Direction.FORWARD);


        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean stopArm = false;
        tStart = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double armPower = 0.0;
            double linearPower = 0.0;
            double collectorPower = 0.0;
            double leftPower = 0.0;
            double rightPower = 0.0;
            double liftPower = 0.0;
            /*
            if ((gamepad1.a) && (System.currentTimeMillis() - tStart > 500)) {
                goDown(-0.4);
                tStart = System.currentTimeMillis();

            }
*/




            if (gamepad2.a) {
                goDown(-0.4);
            }
            if (gamepad2.y) {
                goUp(0.4);

            }


            /*            if (gamepad1.right_bumper) { // go up
                goStraight(COUNTS_PER_MOTOR_REV, 0.6);
            }
            if (gamepad1.left_bumper) { // go down
                goStraight(-COUNTS_PER_MOTOR_REV, 0.6);
            }
*/

            if ((gamepad1.left_stick_y != 0.0) || (gamepad1.right_stick_y != 0.0)) {
                leftPower = Range.clip(-gamepad1.left_stick_y, -0.9, 0.9);
                rightPower = Range.clip(-gamepad1.right_stick_y, -0.9, 0.9);
            }
            else {
                leftPower = 0.0;
                rightPower = 0.0;
            }


            if (gamepad2.right_stick_y != 0.0) {
                armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPower = Range.clip(gamepad2.right_stick_y, -0.7, 0.7);
            }
            else if (gamepad2.left_stick_y !=0.0){
                armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPower = Range.clip(gamepad2.left_stick_y, -0.6, 0.6);
            }
            else{
             tStart = System.currentTimeMillis();
            }

            if (gamepad2.left_trigger != 0.0) {
                linearPower = gamepad2.left_trigger;
            }
            else if (gamepad2.right_trigger != 0.0) {
                    linearPower = -gamepad2.right_trigger;
                }
                else {
                    linearPower = 0.0;
            }


            if(gamepad1.left_trigger!= 0.0){
                collectorPower = gamepad1.left_trigger*-1;
            }

            if(gamepad1.right_trigger!= 0.0){
                collectorPower = gamepad1.right_trigger;
            }


            if(gamepad2.left_bumper){
                liftPower = 1.0;
            }
            else if(gamepad2.right_bumper){
                liftPower = -1.0;
            }


            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            armDrive.setPower(armPower);
            linearDrive.setPower(linearPower);
            collectorDrive.setPower(collectorPower);
            liftDrive.setPower(liftPower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Arm Encoder val = ", armDrive.getCurrentPosition());
            telemetry.update();
        }
    }
    void goUp(double speed) {
//        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive.setTargetPosition(1900);
        armDrive.setPower(speed);

        while (armDrive.isBusy()) {
            telemetry.addData("arm: ", armDrive.getCurrentPosition() );
            telemetry.update();
            idle();
        }
        armDrive.setPower(0.0);
    }






        void goDown(double speed) {
//        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive.setTargetPosition(0);
        armDrive.setPower(speed);

        while (armDrive.isBusy()) {
            telemetry.addData("go down: ", armDrive.getCurrentPosition() );
            telemetry.update();
            idle();
        }
        armDrive.setPower(0.0);
    }


}

