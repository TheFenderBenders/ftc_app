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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@Autonomous(name="LM2_Autonomous_Claiming")
@Disabled
public class LM2_Autonomous_Claiming extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor liftDrive = null;

    final int HANGING = 1;
    final int ONTHEGROUND = 2;
    final int REVERSED = 3;
    final int TURNED = 4;
    final int STRAIGHTENED = 5;
    final int INDEPOT = 6;
    int state = HANGING;

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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        liftDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        long tStart = System.currentTimeMillis();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            switch(state) {
                case HANGING :
                    if (System.currentTimeMillis() - tStart < 8700) {
                        liftDrive.setPower(1.0);
                    } else {
                        liftDrive.setPower(0.0);
                        state = ONTHEGROUND;
                        tStart = System.currentTimeMillis();
                    }
                    break;

                case ONTHEGROUND:
                    if (System.currentTimeMillis() - tStart < 350) {
                        leftDrive.setPower(-0.5);
                        rightDrive.setPower(0.5);
                    } else {
                        leftDrive.setPower(0.0);
                        rightDrive.setPower(0.0);
                        state = REVERSED;
                        tStart = System.currentTimeMillis();
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
                    if (System.currentTimeMillis() - tStart < 250) {
                        leftDrive.setPower(1.0);
                        rightDrive.setPower(-1.0);
                    } else {
                        leftDrive.setPower(0.0);
                        rightDrive.setPower(0.0);
                        state = STRAIGHTENED;
                        tStart = System.currentTimeMillis();
                    }
                    break;

                case STRAIGHTENED:
                    if (System.currentTimeMillis() - tStart < 1100) {
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
                    }
                    break;

                }
                idle();
            }
                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", 0.5, 0.5);
                telemetry.update();
            }

        }




