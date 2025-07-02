package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Slides {
      DcMotorEx slideLeft, slideRight;
    private Gamepad gamepad2;

    public Slides (HardwareMap hardwareMap, Gamepad gamepad){
        gamepad2 = gamepad;
        slideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        slideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");

        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Resets the slides encoder values to zero
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runSlides(){
        /// Controller and coefficients for PID which determine how the motor will behave which we get from tuning
        //This is then done using the arm position, ticks per degree of the motor and the calculations from the controller
        //P (proportional) - proportionate the motor based on the target,
        // I (integral) - integrates the error in order to get rid of smaller steady state errors over time,
        // D (derivative) -considers the rate of change and dampens oscillations which in practice makes it raise slower so it doesn't miss the target
        // F (Feedfoward) - is added to compensate for gravity which is later dictated by the angle that our arm is at

        PIDController controller;
        double p1 = 0.006, i1 = 0.01, d1 = 0.00005;

        double f1 = 0.005;
        int armPos = slideLeft.getCurrentPosition();
        int target1 = armPos;
        controller = new PIDController(p1,i1,d1);
        double ticks_in_degrees = 1440/180;

        controller.setPID(p1,i1,d1);
        //The calculation done here are based off a equation I can't put in here but I do suggest looking up for better understanding
        //The feedfoward part or ff is done by using the cos of target/ticks in radians then multiplying it by our f coefficient
        double pid = controller.calculate(armPos, target1);
        double ff = Math.cos(Math.toRadians(target1/ticks_in_degrees)) * f1;

        double power = pid + ff;
        //Allows for joy stick control
        if(gamepad2.right_stick_y <= 1.0 && gamepad2.right_stick_y != 0.0|| gamepad2.right_stick_y >= -1.0 && gamepad2.right_stick_y != 0){
            // Test this condition later - if(gamepad2.left_stick_y != 0.0) - Again this condition above ^ is terrible but works
            power = gamepad2.right_stick_y * 0.5;
            target1 = armPos;
        }

        slideLeft.setPower(power);
        slideRight.setPower(power);
    }
    public int getSlidePos(){
        return slideLeft.getCurrentPosition();
    }
}
