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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="linearSlideEncoderTest", group="Linear Opmode")
@Disabled
public  class linearSlideEncoderTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor linearDrive = null;

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

        linearDrive = hardwareMap.get(DcMotor.class, "linear_drive");
        linearDrive.setDirection(DcMotor.Direction.FORWARD);
        linearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Mode", "waiting for start");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean stopArm = false;

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
                goldMineral(0.2);
            }



            /*            if (gamepad1.right_bumper) { // go up
                goStraight(COUNTS_PER_MOTOR_REV, 0.6);
            }
            if (gamepad1.left_bumper) { // go down
                goStraight(-COUNTS_PER_MOTOR_REV, 0.6);
            }
*/



            if (gamepad2.left_trigger != 0.0) {
                linearPower = gamepad2.left_trigger;
            }
            else if (gamepad2.right_trigger != 0.0) {
                    linearPower = -gamepad2.right_trigger;
                }
                else {
                    linearPower = 0.0;
            }


            linearDrive.setPower(linearPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }


        void goldMineral(double speed){
            //The method for extending the linear slide and bringing the arm up, all in one button
            linearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearDrive.setTargetPosition(1000);
            linearDrive.setPower(speed);

            while (linearDrive.isBusy()) {
                telemetry.addData("linear slide: ", linearDrive.getCurrentPosition() );
                telemetry.update();
                idle();
            }
            linearDrive.setPower(0.0);
        }

    void silverMineral(double speed) {
        //The method for extending the linear slide and bringing the arm up, all in one button


    }



}

