// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

@Autonomous(name="Drive Encoder", group="Exercises")
//@Disabled
public class encoder_Test extends LinearOpMode
{
    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // reset encoder count kept by left motor.
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // set right motor to run without regard to an encoder.
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set left motor to run to target encoder position and stop with brakes on.
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set left motor to run to target encoder position and stop with brakes on.
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set left motor to run for 5000 encoder counts.

        rightMotor.setTargetPosition(-200);

        // set both motors to 25% power. Movement will start.

        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);

        // wait while opmode is active and left motor is busy running to position.

        while (opModeIsActive() && rightMotor.isBusy())
        {
            telemetry.addData("encoder-fwd", leftMotor.getCurrentPosition() + "  busy=" + leftMotor.isBusy());
            telemetry.update();
            idle();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        // wait 5 sec to you can observe the final encoder position.

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-fwd-end", rightMotor.getCurrentPosition() + "  busy=" + rightMotor.isBusy());
            telemetry.update();
            idle();
        }

        // set position for back up to starting point. In this example instead of
        // having the motor monitor the encoder we will monitor the encoder ourselves.

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor.setTargetPosition(0);

        leftMotor.setPower(0.25);
        rightMotor.setPower(0.25);

        while (opModeIsActive() && rightMotor.getCurrentPosition() < rightMotor.getTargetPosition())
        {
            telemetry.addData("encoder-back", rightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // set motor power to zero to stop motors.

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-back-end", rightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}