package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class PID extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        //Initialise our motor
        DcMotor arm = hardwareMap.dcMotor.get("arm_motor");
        //Run without encoder gets rid of velo limit
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Initialise our PID variables

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        int kP = 2; //constant of proportionality
        int kD = 6; //constant of derivation
        int kI = 1; //constant of integration
        int windupClamp = 1; //Integral Windup switch
        int target = 800; //target
        int error; //Target - current state
        int lastError = target; //error recorded last cycle
        double output=1, integralSum = 0; //motor output and running total of error
        int tolerance = 8; //accepted tolerance from target


        while(opModeIsActive()) {
            error = target - arm.getCurrentPosition();
            while (Math.abs(error) > tolerance) {
                //Error = reference - measured
                error = target - arm.getCurrentPosition();
                //Proportional = error * constant of proportionality
                double proportional = error * kP;
                //Integral is a running sum of error * cycle time
                integralSum += error * timer.time();
                //Integral windup clamp:
                //If the integral output has the opposite sign as the error: turn off integral
                //This will make sure the integral part doesn't cause drastic overshoots
                if ((integralSum > 0 && error < 0) || (integralSum < 0 && error > 0)) {
                    windupClamp = 0;
                } else {
                    windupClamp = 1;
                }
                //Apply the windup clamp and multiply by constant of integration
                double integral = (integralSum * windupClamp) * kI;
                //Derivative - rate of change
                double derivative = (error - lastError) / timer.time();
                lastError = error;
                //Multiply by constant of derivation
                derivative *= kD;
                //Output - the sum of all three parts
                //Used a scaling factor of 1000 so outputs would make sense in the range of -1 to 1
                output = (proportional + integral + derivative) / 100;

                arm.setPower(output);

                timer.reset();

                telemetry.addData("Encoder: ", arm.getCurrentPosition());
                telemetry.addData("Power: ", output);
                telemetry.update();

            }
            arm.setPower(0);
        }



    }
}
