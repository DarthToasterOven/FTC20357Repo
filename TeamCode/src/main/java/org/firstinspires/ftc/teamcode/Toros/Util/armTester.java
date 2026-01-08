package org.firstinspires.ftc.teamcode.Toros.Util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.BatteryClass;
@Disabled
@TeleOp(name = "ArmTester")
public class armTester extends LinearOpMode {
    private DcMotor pivot,slideLeft,slideRight;
    @Override
    public void runOpMode() throws InterruptedException {
        pivot = hardwareMap.get(DcMotorEx.class,"pivot");
        slideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        slideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        waitForStart();
        if(opModeIsActive()){
            while (opModeIsActive()){
                slideLeft.setPower(gamepad2.left_stick_y);
                slideRight.setPower(gamepad2.left_stick_y);


                BatteryClass battery = new BatteryClass(hardwareMap);
                telemetry.addData("Left Power", slideLeft.getPower());
                telemetry.addData("Right Power", slideRight.getPower());
                telemetry.addData("Battery", battery.getBatteryPercent());
                telemetry.update();
            }
        }
    }
}
