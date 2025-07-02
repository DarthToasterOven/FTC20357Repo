package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DriveTrain {

    private DcMotor FrontLeftMotor,BackLeftMotor,FrontRightMotor,BackRightMotor; //Motors
    private boolean Rtoggle,Xtoggle; // Toggles for turning down the speed of the robot
    Gamepad currentGamepad1 = new Gamepad(), previousGamepad1 = new Gamepad(); //fragment of the toggles. needed just in case
    Gamepad gamepad1; // the gamepad which we intialize when we construct the class in the actual drive program
    public DriveTrain(HardwareMap hardwareMap,Gamepad gamepad){
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        BackRightMotor = hardwareMap.get(DcMotor.class, "br");

        // Sets the zero power behaviors which when it stops will resist force on it causing it to stop faster
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //These two motors need to be in reverse in order for correct movement
        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //creates our gamepad object in order to use controls
        this.gamepad1 = gamepad;

    }

    public void drive (){
        currentGamepad1.copy(gamepad1);
        //previousGamepad1.copy(currentGamepad1)

        //Our toggle to slow down either rotationally or just in the x or y directions
        if(currentGamepad1.xWasPressed()){
            Xtoggle = !Xtoggle;
        }
        if(currentGamepad1.bWasPressed()){
            Rtoggle = !Rtoggle;
        }

/*
        if(currentGamepad1.x && !previousGamepad1.x){
            Xtoggle = !Xtoggle;
        }
        if(currentGamepad1.b && !previousGamepad1.b){
            Rtoggle = !Rtoggle;
        }
*/

        //Taking our gamepad inputs
        double x = gamepad1.left_stick_x * 1.1; // the *1.1 counteracts imperfect strafing
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        //Toggles just slow down the speed to 3/4 of the robots speed/power
        if(Xtoggle){
            x *= 0.75;
            y *= 0.75;
        }
        else{
            x*=1;
            y*=1;
        }
        if(Rtoggle){
            turn *= 0.75;
        }
        else{
            turn*=1;
        }




        //Drive variables used in the calculations to run our motors
        double theta = Math.atan2(y, x); //Calculates the angle in radians of the joystick using y and x. It also uses the full 2pi radians so 0.5 ,0.5 = 0.785
        double power = Math.hypot(x, y); // takes the hypotenuse of those angles so 0.5 ,0.5 = 0.7
        double sin = Math.sin(theta - Math.PI / 4); // Sin of the angle in radians
        double cos = Math.cos(theta - Math.PI / 4); // cos of the angle in radians
        double max = Math.max(Math.abs(sin), Math.abs(cos)); //a max that scales the motors so that they are at max power
        //Calculations for our drive motors
        double fl = (power * cos / max + turn);
        double fr = (power * sin / max - turn);
        double bl = (power * sin / max + turn);
        double br = (power * cos / max - turn);

        //If the power were to exceed 1 then it scales down until they don't exceed 1
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

    //Any getter methods that we need in order for telemetry or other use
    public boolean getXToggle(){
        return Xtoggle;
    }
    public boolean getYToggle(){
        return Rtoggle;
    }


}
