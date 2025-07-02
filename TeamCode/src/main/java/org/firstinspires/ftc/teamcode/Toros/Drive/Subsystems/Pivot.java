package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Pivot {
    private DcMotorEx pivot;
    private Gamepad gamepad2;
    public Pivot(HardwareMap hardwareMap, Gamepad gamepad){
        gamepad2 = gamepad; // gamepad for controls
        pivot = hardwareMap.get(DcMotorEx.class,"pivot");
        // resets and starts the encoder so we start at 0
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runPivot(){
        /// Controller and coefficients for PID which determine how the motor will behave which we get from tuning
        //This is then done using the arm position, ticks per degree of the motor and the calculations from the controller
        //P (proportional) - proportionate the motor based on the target,
        // I (integral) - integrates the error in order to get rid of smaller steady state errors over time,
        // D (derivative) -considers the rate of change and dampens oscillations which in practice makes it raise slower so it doesn't miss the target
        // F (Feedfoward) - is added to compensate for gravity which is later dictated by the angle that our arm is at
        PIDController controller;
        double p1 = 0.05, i1 = 0.001, d1 = 0.00003;
        int armPos = pivot.getCurrentPosition();
        double f1 = -0.04;
        int target1 = armPos;
        controller = new PIDController(p1,i1,d1);
        double ticks_in_degrees = 1440/180;

        controller.setPID(p1,i1,d1);
        //The calculation done here are based off a equation I can't put in here but I do suggest looking up for better understanding
        //The feedfoward part or ff is done by using the cos of target/ticks in radians then multiplying it by our f coefficient
        double pid = controller.calculate(armPos, target1);
        double ff = Math.cos(Math.toRadians(target1/ticks_in_degrees)) * f1;

        double power = pid + ff; //Calculates the power of the motor
        //Below is if we need to control the arm through the joystick rather than preprogrammed buttons
        if(gamepad2.left_stick_y <= 1.0 && gamepad2.left_stick_y != 0.0|| gamepad2.left_stick_y >= -1.0 && gamepad2.left_stick_y != 0){ //straight up this condition is garbage and should be changed as it makes no sense but it just works
            // Test this condition later - if(gamepad2.left_stick_y != 0.0)
            power = gamepad2.left_stick_y * 0.5;
            target1 = armPos;
        }
//        if (pivot.getCurrentPosition() < 60) {
//            f1 = 0.15;
//        } else if (pivot.getCurrentPosition() > 60 && pivot.getCurrentPosition() < 100) {
//            f1 = 0.0001;
//
//        }else if (pivot.getCurrentPosition() > 60) {
//            f1 = -0.15;
//        }
        if(pivot.getCurrentPosition() < -240 || pivot.getCurrentPosition() >-240){
            f1 *=-1;
        }



        pivot.setPower(power);
    }
}
