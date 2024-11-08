package org.firstinspires.ftc.teamcode.Toros.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
@Autonomous(name = "AutoScrim1")
//Autonomous for first meet of 2024/25
public class AutoScrim1 extends LinearOpMode{
    private DcMotor FrontLeftMotor;
    private DcMotor BackLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackRightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
            //power motors
            //go right
            runMotors(1,-1,-1,1,1000);
            runMotors(1,-1,-1,1,1000);


    }
    private void initHardware(){
        //Motors
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    private void runMotors(int p1, int p2, int p3, int p4, int sleepTime){
        FrontLeftMotor.setPower(p1);
        FrontRightMotor.setPower(p2);
        BackLeftMotor.setPower(p3);
        BackRightMotor.setPower(p4);
        sleep(sleepTime);
    }
}
