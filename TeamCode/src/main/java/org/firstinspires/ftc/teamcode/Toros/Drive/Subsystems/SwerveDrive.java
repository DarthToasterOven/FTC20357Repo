package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwerveDrive {
    private DcMotor topLeft,topRight,bottomLeft,bottomRight;
    Gamepad gamepad1;

    public SwerveDrive(HardwareMap hardwareMap, Gamepad gamepad){
        gamepad1 = gamepad;
        topLeft = hardwareMap.get(DcMotor.class, "fl");
        topRight = hardwareMap.get(DcMotor.class, "bl");
        bottomLeft = hardwareMap.get(DcMotor.class, "fr");
        bottomRight = hardwareMap.get(DcMotor.class, "br");

        // Sets the zero power behaviors which when it stops will resist force on it causing it to stop faster
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void drive(){

    }

}
