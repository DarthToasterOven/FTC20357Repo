package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Toros.Util.BatteryClass;

@TeleOp(name = "MainDrive")
public class MainDrive extends LinearOpMode {

    /**
     * [PIDF controller] PIDF is a closed loop control which takes a proportional, integral, and derivative terms to calculate the error
     * as a difference of a target and will correct based on the terms.We use this to have precise control of our arm for our intake.
     **/

    private PIDController controller;
    private boolean Rtoggle,Xtoggle,breakfast;
    //
    public static double p = 0.004, i = 0.001, d = 0.0005;
    public static double f = 0.195;

//     P is for proportional which will be proportionate to the error this causes the arm to go up for us
//     I is for integral which integrates past values of error seeking to reduced the residual error by adding control and eliminate the error which gets us closer to the target point
//     D is for derivative which best estimates the trend of the error based on the rate of change to reduced the effect to dampen it to not overshoot
//     F is for feedforward which accounts for things more external and prevents disturbances in our use case showing gravity who is boss

    public static int target = 50;
    private final double ticks_in_degrees = 1440 / 180; // Ticks of the tetrix 60:1 motor in degrees (divided by 180)
    //Declares the Variables for all of our motors and servos
    private DcMotor FrontLeftMotor,BackLeftMotor,FrontRightMotor,BackRightMotor; //Motors
    private  DcMotorEx pivot, slideLeft, slideRight;
    private VoltageSensor volt_prime;
    private Servo specClaw, budget;
    private CRServo sampClaw;
    Gamepad currentGamepad1 = new Gamepad(), previousGamepad1 = new Gamepad(); //Gamepads used to make toggles
    Gamepad currentGamepad2 = new Gamepad(), previousGamepad2 = new Gamepad();
    @Override
    public void runOpMode() throws InterruptedException {
//        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Initializing Hardware in method down below
        initHardware();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                drive();
                //runPivot();
                runSlides();
                claw();

                ///Battery power
                double volts = volt_prime.getVoltage();
                double battery = 0;
                if (volts > 12.00) {
                    battery = 100;
                } else if (volts <= 12) {
                    battery = (volts / 12.00) * 100;
                }

                telemetry.addData("Battery%", battery);

                initTelemetry();
                telemetry.update();

            }
        }
    }

    private void initHardware () {

        //Motors
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        BackRightMotor = hardwareMap.get(DcMotor.class, "br");
        pivot = hardwareMap.get(DcMotorEx.class,"pivot");
        slideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        slideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        sampClaw = hardwareMap.get(CRServo.class,"slurp");
        specClaw = hardwareMap.get(Servo.class,"specClaw");
        budget = hardwareMap.get(Servo.class,"brisket");

        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Zero Power Behaviors
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        volt_prime = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    private void initTelemetry () {

        BatteryClass battery = new BatteryClass(hardwareMap);
        telemetry.addData("Battery", battery.getBatteryPercent());
        telemetry.addData("Direct Control",breakfast);
        telemetry.addData("Toggle",Xtoggle);
        telemetry.addData("Toggle",Rtoggle);
        telemetry.update();
    }
    private void drive () throws InterruptedException {


        if(currentGamepad1.x && !previousGamepad1.x){
            Xtoggle = !Xtoggle;
        }
        if(currentGamepad1.b && !previousGamepad1.b){
            Rtoggle = !Rtoggle;
        }






        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        if(Xtoggle){
            x *= 0.75;
        }
        else{
            x*=1;
        }
        if(Rtoggle){
            turn *= 0.75;
        }
        else{
            turn*=1;
        }




        //Drive variables used in the calculations to run our motors
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        /**
         In basics this is taking the x and y of the left stick making them into an angle
         with the power being the hypot which is the square root of the sum of squares of the inputs
         more info here https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Math/hypot
         then takes the sin and cos of the angle making sure to convert to radians. It then creates a max
         using the absolute value of the sin and cos.

         The idea is that where you are going is angle theta with each wheel being a vector and when combined make the target direction when rotated 45 degrees

         Found on YT www.youtube.com/watch?v=gnSW2QpkGXQ which is a video about coding for mecanum drive wheels
         */


        //Calculations for our drive motors

        double fl = (power * cos / max + turn);
        double fr = (power * sin / max - turn);
        double bl = (power * sin / max + turn);
        double br = (power * cos / max - turn);

        /**
         In continuation the power is then calculated with the angles multiplied by the sin or cos divided the difference or sum of the max and turn
         */

        //If statement below is to make sure one motor does not exceed the power limit making it scale down

        if ((power + Math.abs(turn)) > 1) {
            fl /= power + Math.abs(turn);
            fr /= power + Math.abs(turn);
            bl /= power + Math.abs(turn);
            br /= power + Math.abs(turn);
        }



        //Motor Drive
        FrontLeftMotor.setPower(fl);
        FrontRightMotor.setPower(fr);
        BackLeftMotor.setPower(bl);
        BackRightMotor.setPower(br);
    }
    private void claw(){
        if(gamepad2.left_trigger > 0){
            sampClaw.setPower(gamepad2.left_trigger);
        }
        if(gamepad2.right_trigger > 0){
            sampClaw.setPower(-gamepad2.right_trigger);
        }
        if(gamepad2.b){
            sampClaw.setPower(0);
        }
        if(gamepad2.left_bumper){
            specClaw.setPosition(0);
        }
        if(gamepad2.right_bumper){
            specClaw.setPosition(1);
        }
        if(currentGamepad2.y && !previousGamepad2.y){
            breakfast = !breakfast;
        }

        if(breakfast){
            budget.setPosition(0);
        }
        else{
            budget.setPosition(1);
        }
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
            f *=-1;
        }



        pivot.setPower(power);
    }
    public void runSlides(){
        PIDController controller;
        double p1 = 0.006, i1 = 0.01, d1 = 0.00005;

        double f1 = 0.005;
        int armPos = slideLeft.getCurrentPosition();
        int target1 = armPos;
        controller = new PIDController(p1,i1,d1);
        double ticks_in_degrees = 1440/180;

        controller.setPID(p1,i1,d1);

        double pid = controller.calculate(armPos, target1);
        double ff = Math.cos(Math.toRadians(target1/ticks_in_degrees)) * f1;

        double power = pid + ff;
        if(gamepad2.right_stick_y <= 1.0 && gamepad2.right_stick_y != 0.0|| gamepad2.right_stick_y >= -1.0 && gamepad2.right_stick_y != 0){
            power = gamepad2.right_stick_y * 0.5;
            target1 = armPos;
        }

        slideLeft.setPower(power);
        slideRight.setPower(power);
    }
}



//:3