package org.firstinspires.ftc.teamcode.Toros.Autonomous.RR;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoV1")
@Disabled
//Autonomous for first meet of 2024/25
public class AutoV1 extends LinearOpMode{
    private DcMotor FrontLeftMotor;
    private DcMotor BackLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackRightMotor;

    private PIDController controller;
    private PIDController controller2;



    public static double p1 = -0.04, i1 = 0.001, d1 = 0.0005;

    public static double f1 = 0.185;

    public static double p2 = 0.0, i2 = 0.0, d2 = 0.0;

    public static double f2 = 0;

    public static int target2 = 250;

    private final double ticks_in_degrees = 1440/180;
    private DcMotorEx slideLeft,slideRight,pivot;
    private Servo fingers,wrist,elbow;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        //power motors
//        runMotors(0.5,0.5,0.5,0.5,650);
//        runSlides(650);

        runMotors(1, -0.75, -0.75, 1, 1200);
//        runSlides(-425);
//        runPivot(0);

    }
    private void initHardware(){
        //Motors
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        BackRightMotor = hardwareMap.get(DcMotor.class, "br");
        slideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        fingers = hardwareMap.get(Servo.class,"fingers");
        wrist = hardwareMap.get(Servo.class,"wrist");
        elbow = hardwareMap.get(Servo.class,"elbow");
        pivot = hardwareMap.get(DcMotorEx.class,"pivot");
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fingers.setPosition(1);
        elbow.setPosition(0);
        wrist.setPosition(0);
    }
    private void runMotors(double p1, double p2, double p3, double p4, int sleepTime){
        FrontLeftMotor.setPower(p1);
        FrontRightMotor.setPower(p2);
        BackLeftMotor.setPower(p3);
        BackRightMotor.setPower(p4);
        sleep(sleepTime);
    }
    public void runSlides(int target1){

        PIDController controller;
        double p1 = 0.006, i1 = 0.01, d1 = 0.00005;

        double f1 = 0.005;
        int armPos = slideLeft.getCurrentPosition();
        controller = new PIDController(p1,i1,d1);
        double ticks_in_degrees = 1440/180;

        controller.setPID(p1,i1,d1);

        double pid = controller.calculate(armPos, target1);
        double ff = Math.cos(Math.toRadians(target1/ticks_in_degrees)) * f1;

        double power = pid + ff;

        slideLeft.setPower(power);
        slideRight.setPower(power);
    }
    public void runPivot(int target1){

        PIDController controller;
        double p1 = 0.05, i1 = 0.001, d1 = 0.00003;
        int armPos = pivot.getCurrentPosition();
        double f1 = -0.04;

        controller = new PIDController(p1,i1,d1);
        double ticks_in_degrees = 1440/180;

        controller.setPID(p1,i1,d1);

        double pid = controller.calculate(armPos, target1);
        double ff = Math.cos(Math.toRadians(target1/ticks_in_degrees)) * f1;

        double power = pid + ff;
//        if (pivot.getCurrentPosition() < 60) {
//            f1 = 0.15;
//        } else if (pivot.getCurrentPosition() > 60 && pivot.getCurrentPosition() < 100) {
//            f1 = 0.0001;
//
//        }else if (pivot.getCurrentPosition() > 60) {
//            f1 = -0.15;
//        }
        pivot.setPower(power);
    }
}
