package org.firstinspires.ftc.teamcode;
//high basket
//linear 2150
//wrist ver. 0.8
//wristhor 0.15
//arm 2800
//
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;
@TeleOp

public class MecanumWheelArm extends LinearOpMode{
    //arm servo+motor declaration
    public DcMotor  armMotor         = null; //the arm motor
    public Servo    claw             = null; //the active claw servo
    public Servo    wrist_horizontal = null; //the wrist horizontal servo
    public Servo    wrist_vertical   = null; //the wrist vertical servo
    // drivetrain wheel motor declaration
    private DcMotor wheel_0=null;
    private DcMotor wheel_1=null;
    private DcMotor wheel_2=null;
    private DcMotor wheel_3=null;
    private DcMotor linearR=null;
    private DcMotor linearL=null;
    //encoder ticks per degree rotation of the arm
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation
    //arm position variables
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;
    final double claw_COLLECT    = 0.3;
    final double claw_DEPOSIT    = 0.55;
    final double wrist_vertical_FOLDED_IN   = 1;
    final double wrist_vertical_FOLDED_OUT  = 0.5;
    //adjust this variable to change how much the arm adjudsts by for the left and right triggers.
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;
    //variables used to set the arm to a specific position.
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    //variable for if wrist_vertical is out or not(true is in, false is out)
    boolean wrist_verticalLocation=true;
    //varaibles for wrist_horizontal
    //change wrist_horizontalShift to change how fast the wrist_horizontal moves.
    final double wrist_verticalShift = 0.05;
    double wrist_verticalPos=wrist_vertical_FOLDED_IN;
    double wrist_verticalmove=wrist_vertical_FOLDED_IN;
    //variables for wrist_horizontal
    final double wrist_horizontalShift = 0.05;
    double wrist_horizontalPos=0.2;
    double wrist_horizontalmove=0.2;
    //variable for where the linear slides are
    double linearpos=0;
    //how fast the linear slide moves
    final double LINEARSHIFT=20;
    //Change the constant to change how fast the arms moves during manual
    final double armShift=2*ARM_TICKS_PER_DEGREE;
    //lowest arm motor speed
    final int LOW_ARM_SPEED=2100;
    //highest arm motor speed
    final int HIGH_ARM_SPEED=4100;
    //acceleration/deceleration speed
    final int ARM_ACCELERATION=250;
    //Variables to make joystick presses not trigger constantly
    boolean leftStickPressed=false;
    boolean rightStickPressed=false;
    //Variable for initiating manual
    boolean manual_init=false;
    
    //main loop
    @Override
    public void runOpMode() {
        //initiate linear motors
        linearR=hardwareMap.get(DcMotor.class,"linearR");
        linearL=hardwareMap.get(DcMotor.class,"linearL");

        linearL.setDirection(DcMotor.Direction.REVERSE);
        //initiate drivetrain motors
        wheel_0   = hardwareMap.get(DcMotor.class, "wheel_0");
        wheel_1    = hardwareMap.get(DcMotor.class, "wheel_1");
        wheel_2   = hardwareMap.get(DcMotor.class, "wheel_2");
        wheel_3   = hardwareMap.get(DcMotor.class, "wheel_3");
        
        wheel_2.setDirection(DcMotor.Direction.REVERSE);
        wheel_3.setDirection(DcMotor.Direction.REVERSE);
        //initiate arm motor
        armMotor   = hardwareMap.get(DcMotor.class, "arm_lift"); //the arm motor
        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        //tune arm motor current alert for auto claw realease(between 4-5)
        ((DcMotorEx) armMotor).setCurrentAlert(10,CurrentUnit.AMPS);
        ((DcMotorEx) linearR).setCurrentAlert(5,CurrentUnit.AMPS);
        ((DcMotorEx) linearL).setCurrentAlert(5,CurrentUnit.AMPS);
        //reset encoder
        // armMotor.setTargetPosition(0);
        // armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // linearR.setTargetPosition(0);
        // linearR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // linearR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // linearL.setTargetPosition(0);
        // linearL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // linearL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /* Define and initialize servos.*/
        claw = hardwareMap.get(Servo.class, "servo_claw");
        wrist_horizontal  = hardwareMap.get(Servo.class, "servo_horizontal");
        wrist_vertical=hardwareMap.get(Servo.class,"servo_vertical");
        claw.setPosition(claw_DEPOSIT);
        wrist_vertical.setPosition(wrist_vertical_FOLDED_IN);
        wrist_horizontal.setPosition(wrist_horizontalPos);
        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        //wait for driver to press play
        waitForStart();
        //repeat untill opmode ends
        //claw position
        double clawpos=claw_DEPOSIT;
        while (opModeIsActive()){
            if (gamepad2.a) {
                //close claw
                clawpos=claw_COLLECT;
                telemetry.addData("Claw:","Collect");
            }
            else if (gamepad2.b) {
                //open claw
                clawpos=claw_DEPOSIT;
                telemetry.addData("Claw: ","Deposit");
            }
            //telemetry if motor exceeded current limit
            if (((DcMotorEx) armMotor).isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }
            //apply claw position
            claw.setPosition(clawpos);
            //Manual code
            //wrist_horizontal movement
            if (gamepad2.dpad_up){
                wrist_verticalmove-=wrist_verticalShift;
            }
            if (gamepad2.dpad_down){
                wrist_verticalmove+=wrist_verticalShift;
            }
            //wrist_vertical movement
            if (gamepad2.dpad_left){
                wrist_horizontalmove-=wrist_horizontalShift;
            }
            if (gamepad2.dpad_right){
                wrist_horizontalmove+=wrist_horizontalShift;
            }
            //arm movement
            double armmove=gamepad2.left_stick_y;
            //linear slide movement
            double linearMove=gamepad2.right_stick_y;
            //make sure the wrist_vertical is within range
            if (wrist_verticalmove<=1 && wrist_verticalmove>=0.07){
                wrist_verticalPos=wrist_verticalmove;
            }
            else if (wrist_verticalmove>1){
                wrist_verticalPos=1;
                wrist_verticalmove=1;
            }
            else if (wrist_verticalmove<0.07){
                wrist_verticalPos=0.07;
                wrist_verticalmove=0.07;
            }
            //make sure wrist_horizontal is within range
            if (wrist_horizontalmove<=1 && wrist_horizontalmove>=0){
                wrist_horizontalPos=wrist_horizontalmove;
            }
            else if (wrist_horizontalmove>1){
                wrist_horizontalPos=1;
                wrist_horizontalmove=1;
            }
            else if (wrist_horizontalmove<0){
                wrist_horizontalPos=0;
                wrist_horizontalmove=0;
            }
            //arm positions
            if (gamepad2.right_bumper){
                //arm 2240
                //wristverti 0.85
                //wristhoriz 0.35
                //linear 2220
                /* This is the correct height to score the sample in the HIGH BASKET */
                armPosition = 2240;
                linearpos=2220;
                wrist_verticalPos=0.85;
                wrist_horizontalPos=0.35;
            }
            else if (gamepad2.y){
                //reset arm
                linearpos=100;
                if (armPosition>1994){
                    armPosition=1994;
                }
            }
            else if (gamepad2.left_bumper){
                //this is the hight for the specimen
                //arm 4467
                //wriat ver 0.5
                //wrist hor 0.35
                //linear 100
                armPosition=3950;
                wrist_verticalPos=0.5;
                wrist_horizontalPos=1;
                linearpos=75;
            }
            else if (gamepad1.right_bumper){
            //arm 4090
            //wrist ver=0.85
            //wrist hor=0.4;
            //linear=100;
                armPosition=4090;
                wrist_horizontalPos=0.4;
                wrist_verticalPos=0.85;
                linearpos=100;
            }
            //set wrist_vertical to opposite position
            else if (gamepad2.right_stick_button && !rightStickPressed){
                if (wrist_verticalPos>0.6666){
                    wrist_verticalLocation=false;
                }
                else{
                    wrist_verticalLocation=true;
                }
                if (wrist_verticalLocation){
                    wrist_verticalPos=wrist_vertical_FOLDED_IN;
                    wrist_verticalLocation=false;
                }
                else if (!wrist_verticalLocation){
                    wrist_verticalPos=wrist_vertical_FOLDED_OUT;
                    wrist_verticalLocation=true;
                }
                rightStickPressed=true;
            }
            //for not having the left & right button trigger constantly while held down
            if (!gamepad2.left_stick_button){
                leftStickPressed=false;
            }
            if (!gamepad2.right_stick_button){
                rightStickPressed=false;
            }
            //set arm position
            if (armPosition+armShift*armmove<=5000 && armPosition+armShift*armmove>=0){
                armPosition=armPosition+armShift*armmove;
            }
            else if (armPosition+armShift*armmove>5000){
                armPosition=5000;
            }
            else if (armPosition+armShift*armmove<0){
                armPosition=0;
            }
            //set linear slide position
            if (linearpos+linearMove*LINEARSHIFT*-1<75){
                linearpos=75;
            }
            else{
            linearpos=linearpos+linearMove*LINEARSHIFT*-1;
            }
            telemetry.addData("linear Target:",linearpos);
            linearR.setTargetPosition((int) (linearpos));
            linearL.setTargetPosition((int) (linearpos));
            ((DcMotorEx) linearR).setVelocity(2000);
            ((DcMotorEx) linearL).setVelocity(2000);
            linearR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //set wrist_vertical position
            wrist_vertical.setPosition(wrist_verticalPos);
            //set wrist_horizontal position
            wrist_horizontal.setPosition(wrist_horizontalPos);
            /* Here we set the target position of our arm to match the variable that was selected
            by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
            armMotor.setTargetPosition((int) (armPosition));
            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //teletmetry log
            telemetry.addData("wrist_verticalmove:",wrist_verticalPos);
            telemetry.addData("wrist_horizontalmove:",wrist_horizontalPos);
            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            
            //below is drivetrain
            // Mecanum drive is controlled with three axes: drive (front-and-back),
            // strafe (left-and-right), and twist (rotating the whole chassis).
            double drive  = gamepad1.left_stick_y*0.7;
            final double strafe_speed=0.5;
            double strafe = -gamepad1.left_stick_x*0.5;
            if (gamepad1.dpad_left){
                strafe=strafe_speed;
            }
            else if (gamepad1.dpad_right){
                strafe=-strafe_speed;
            }
            double twist  = -gamepad1.right_stick_x;
            telemetry.addData("drive: ", drive);
            telemetry.addData("strafe: ", strafe);
            telemetry.addData("twist: ", twist);
            telemetry.update();
            
            double[] speeds = {
                (drive + strafe + twist), //forward-left motor(wheel_0)
                (drive - strafe - twist), //forward-right motor(wheel_1)
                (drive - strafe + twist), //back-left motor(wheel_2)
                (drive + strafe - twist)  //back-right motor(wheel_3)
            };
            
            // Loop through all values in the speeds[] array and find the greatest
            // *magnitude*.  Not the greatest velocity.
            double max = Math.abs(speeds[0]);
            for(int i = 0; i < speeds.length; i++) {
                if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
            }
            
            // If and only if the maximum is outside of the range we want it to be,
            // normalize all the other speeds based on the given speed value.
            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            }
            
            // apply the calculated values to the motors.
            wheel_0.setPower(speeds[0]);
            wheel_3.setPower(speeds[1]);
            wheel_1.setPower(speeds[2]);
            wheel_2.setPower(speeds[3]);
        }
        wrist_vertical.setPosition(wrist_vertical_FOLDED_IN);
    }
}