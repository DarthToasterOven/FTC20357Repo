package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name ="testLaunch")
public class testLaunch extends LinearOpMode {
    private DcMotorEx L1,L2;

    @Override
    public void runOpMode(){
        L1 = hardwareMap.get(DcMotorEx.class,"top");
        L2 = hardwareMap.get(DcMotorEx.class,"bottom");

        L1.setDirection(DcMotorSimple.Direction.REVERSE);
        if(opModeIsActive()){
            while (opModeIsActive()){
                if(gamepad1.left_trigger > 0){
                    L1.setPower(gamepad1.left_trigger);
                    L2.setPower(-gamepad1.left_trigger);
                }

                if(gamepad1.right_trigger > 0){
                    L1.setPower(-gamepad1.right_trigger);
                    L2.setPower(gamepad1.right_trigger);
                }
            }
        }
    }
}
