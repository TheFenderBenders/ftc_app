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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


    @TeleOp(name="LM3 Teleop DONOTDELETE", group="Linear Opmode")
    @Disabled
    public class TestTeleOp extends LinearOpMode {

        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftDrive = null;
        private DcMotor rightDrive = null;
        private DcMotor liftDrive = null;
        private DcMotor armDrive = null;
        private CRServo linearDrive = null;
        private CRServo collectorDrive = null;
        double leftPower = 0.0;
        double rightPower = 0.0;
       //  int RAMPUP = 8;

        @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
            rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
            liftDrive = hardwareMap.get(DcMotor.class, "lift_drive");
            armDrive = hardwareMap.get(DcMotor.class, "arm_drive");
            linearDrive = hardwareMap.get(CRServo.class, "linear_drive");
            collectorDrive = hardwareMap.get(CRServo.class, "collector_drive");

            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
            liftDrive.setDirection(DcMotor.Direction.FORWARD);

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                // Setup a variable for each drive wheel to save power level for telemetry
                double linearslide = 0.0;
                double arm = 0.0;
                double collector = 0.0;

               /* if (gamepad1.left_stick_y < 0 ){ // meaning robot has to move fwd
                    leftPower -= (gamepad1.left_stick_y + leftPower) / RAMPUP;
                }
                else if(gamepad1.left_stick_y > 0){ // meaning robot has to move backward
                    leftPower += (gamepad1.left_stick_y - leftPower) / RAMPUP;
                }
                else{
                    leftPower = 0.0;
                }

                if (gamepad1.right_stick_y < 0 ){ // meaning robot has to move fwd
                    rightPower -= (gamepad1.right_stick_y + rightPower) / RAMPUP;
                }
                else if(gamepad1.right_stick_y > 0){ // meaning robot has to move backward
                    rightPower += (gamepad1.right_stick_y - rightPower) / RAMPUP;
                }
                else{
                    rightPower = 0.0;
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

                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);

                if(gamepad2.right_bumper) {
                    liftDrive.setPower(1.0);
                }
                else if(gamepad2.left_bumper) {
                    liftDrive.setPower(-1.0);
                }
                else {
                    liftDrive.setPower(0.0);
                }

                if(gamepad2.left_trigger>0.0){
                    linearslide = (0.79*gamepad2.left_trigger);
                }
                else if(gamepad2.right_trigger>0.0)
                linearslide = (-0.79*gamepad2.right_trigger);
                else {
                    linearslide = 0.0;
                }
                if(gamepad2.a){
                    linearslide = (0.79);
                }
                else{
                    linearslide  = 0.0;
                }

                if(gamepad2.right_stick_y!=0.0){
                 arm = gamepad2.right_stick_y;
                }
                else{
                    arm = 0.0;
                }

                if(gamepad1.left_trigger>0.0){
               CRServoTurn(50,1.0,collectorDrive);
                }
                else if(gamepad1.right_trigger>0.0) {
                    CRServoTurn(50,-1.0,collectorDrive);
                }
                else {
                    collectorDrive.setPower(0.0);
                }


                armDrive.setPower(arm);
                linearDrive.setPower(linearslide);

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left-drive (%.2f), right-drive(%.2f)", leftPower, rightPower);
                telemetry.addData("linearSlide", linearslide);
                telemetry.addData("collector", 0.79);
                telemetry.update();
            }

        }
        public void CRServoTurn(long sleepTime, double speed, CRServo servo){

                servo.setPower(speed * 0.79);
                sleep(sleepTime);
                idle();
            }

            }




