package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Pivot {
    private DcMotorEx pivot;
    private Gamepad gamepad2;
    public Pivot(HardwareMap hardwareMap, Gamepad gamepad){
        gamepad2 = gamepad;
        pivot = hardwareMap.get(DcMotorEx.class,"pivot");
    }
    public void runPivot(){
        PIDController controller;
        double p1 = 0.05, i1 = 0.001, d1 = 0.00003;
        int armPos = pivot.getCurrentPosition();
        double f1 = -0.04;

        int target1 = armPos;
        controller = new PIDController(p1,i1,d1);
        double ticks_in_degrees = 1440/180;

        controller.setPID(p1,i1,d1);

        double pid = controller.calculate(armPos, target1);
        double ff = Math.cos(Math.toRadians(target1/ticks_in_degrees)) * f1;

        double power = pid + ff;
        if(gamepad2.left_stick_y <= 1.0 && gamepad2.left_stick_y != 0.0|| gamepad2.left_stick_y >= -1.0 && gamepad2.left_stick_y != 0){
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
