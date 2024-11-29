/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous

public class AutoSampleGryo extends OpMode
{
    //arm servo+motor declaration
    public DcMotor  armMotor    = null; //the arm motor
    public CRServo  intake      = null; //the active intake servo
    public Servo    wrist       = null; //the wrist servo
    // drivetrain wheel motor declaration
    private DcMotor LFront=null;
    private DcMotor LRear=null;
    private DcMotor RFront=null;
    private DcMotor RRear=null;
    //linear slide motors
    private DcMotor linearL=null;
    private DcMotor linearR=null;
    //IMU declatration
    private IMU imu=null;
    //initiate runtime
    private ElapsedTime runtime=new ElapsedTime();
    //encoder ticks per degree rotation of the arm
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation
    //arm position variables(to be use for armToPosition in your code)
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;
    //adjust this variable to change how much the arm adjudsts by for the left and right triggers.
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;
    //variables used to set the arm to a specific position.
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double wristPosition = WRIST_FOLDED_IN;
    double intakeSpeed = INTAKE_OFF;
    double linearPos = 0;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //Log robot is initializing
        telemetry.addData("Status:", "Initializing");
        telemetry.update();
        //set the orientation of the REV hub on the robot(the IMU location is on one of the corners of the robot)
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        //initiate imu
        imu = hardwareMap.get(IMU.class, "imu");
        //initiate linear motors
        linearL=hardwareMap.get(DcMotor.class,"linearL");
        linearR=hardwareMap.get(DcMotor.class,"linearR");
        //reverse left side direction
        linearL.setDirection(DcMotor.Direction.REVERSE);
        //brake on stop
        linearL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Set current alert
        ((DcMotorEx) linearR).setCurrentAlert(5,CurrentUnit.AMPS);
        ((DcMotorEx) linearL).setCurrentAlert(5,CurrentUnit.AMPS);
        //Reset encoder
        linearR.setTargetPosition(0);
        linearR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearL.setTargetPosition(0);
        linearL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //initiate drivetrain motors
        LFront   = hardwareMap.get(DcMotor.class, "wheel_2");
        LRear    = hardwareMap.get(DcMotor.class, "wheel_0");
        RFront   = hardwareMap.get(DcMotor.class, "wheel_1");
        RRear    = hardwareMap.get(DcMotor.class, "wheel_3");
        //set the drivetrain motors to brake on stop.
        LFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //set direction
        LFront.setTargetPosition(0);
        LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRear.setTargetPosition(0);
        LRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFront.setTargetPosition(0);
        RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRear.setTargetPosition(0);
        RRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        LFront.setDirection(DcMotor.Direction.REVERSE);
        LRear.setDirection(DcMotor.Direction.REVERSE);
        //initiate arm motor
        armMotor   = hardwareMap.get(DcMotor.class, "arm_lift"); //the arm motor
        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);
        //define servos
        intake = hardwareMap.get(CRServo.class, "servo_intake");
        wrist  = hardwareMap.get(Servo.class, "servo_rotate");
        //Log that initialization is complete.
        telemetry.addData("Status: ", "Initialized");
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        //send telemetry of the gyro heading while waiting for driver to start.
        telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
        telemetry.update();
    }

    /**
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        //Log the robot is initializing servo+motor positions
        telemetry.addData("Status: ","Intializing Positions");
        telemetry.update();
        //set motors to run using encoder ticks.
        LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //reset the IMU
        imu.resetYaw();
        //reset runtime(for future needs, if using time controls)
        runtime.reset();
        //reset arm encoder
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //initialize servos
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);
        //Log position intiation is complete
        telemetry.addData("Status: ","Robot Ready");
        telemetry.update();
        
        //ADD MAIN CODE HERE
        //be sure to use telemetry and log all variables for debugging!
        drivegyro(0, 0, 15, 0.5, 0.1);
        
    }

    /**
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        //Constantly set the arm positions, as the arm motor requires this to operate properly
        armToPosition(armPosition, wristPosition, intakeSpeed,linearPos);
        telemetry.addData("Left Front Motor (2):",Double.toString(LFront.getCurrentPosition()));
        telemetry.addData("Right Front Motor (1):",Double.toString(RFront.getCurrentPosition()));
        telemetry.addData("Left Rear Motor (0):",Double.toString(LRear.getCurrentPosition()));
        telemetry.addData("Right Rear Motor (3):",Double.toString(RRear.getCurrentPosition()));
        telemetry.update();
    }

    /**
     * main gryo drive function.
     * All parameters are cm.
     * Twist is degrees.
     * @param twist is degrees to turn
     * @param strafe is how far sideways
     * @param drive is how far forwards
     * @param speed is drive speed
     * @param rotationspeed is rotation speed
     * @return none
     */
    public void drivegyro(double twist, double strafe, double drive, double speed, double rotationspeed){
        double targetdirection=getHeading()+twist;
        double distances[]=calculateWheelMovement(strafe, drive, twist);
        distances[0]=LFront.getCurrentPosition()+distances[0];
        distances[1]=RFront.getCurrentPosition()+distances[1];
        distances[2]=LRear.getCurrentPosition()+distances[2];
        distances[3]=RRear.getCurrentPosition()+distances[3];
        //wheel one
        LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFront.setPower(speed);
        LFront.setTargetPosition((int)distances[0]);
        //wheel two
        LRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRear.setPower(speed);
        LRear.setTargetPosition((int)distances[2]);
        //wheel three
        RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFront.setPower(speed);
        RFront.setTargetPosition((int)distances[1]);
        //wheel four
        RRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRear.setPower(speed);
        RRear.setTargetPosition((int)distances[3]);
        //run motors
        LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double adjust=5;
        double LFDif=Math.abs(LFront.getCurrentPosition()-distances[0]);
        double LRDif=Math.abs(LRear.getCurrentPosition()-distances[2]);
        double RFDif=Math.abs(RFront.getCurrentPosition()-distances[1]);
        double RRDif=Math.abs(RRear.getCurrentPosition()-distances[3]);
        boolean exit=false;
        while (!exit){
            //get steering correction
            double steeringCorrection=getSteeringCorrection(targetdirection, rotationspeed);
            //set motor power
            RFront.setPower(speed-steeringCorrection);
            RRear.setPower(speed-steeringCorrection);
            LFront.setPower(speed+steeringCorrection);
            LRear.setPower(speed+steeringCorrection);
            //telemetry
            telemetry.addData("Left Front Motor (2):",Double.toString(LFront.getCurrentPosition()));
            telemetry.addData("Left Front Motor Target:",Double.toString(distances[0]));
            telemetry.addData("Right Front Motor (1):",Double.toString(RFront.getCurrentPosition()));
            telemetry.addData("Right Front Motor Target:",Double.toString(distances[1]));
            telemetry.addData("Left Rear Motor (0):",Double.toString(LRear.getCurrentPosition()));
            telemetry.addData("Left Rear Motor Target:",Double.toString(distances[2]));
            telemetry.addData("Right Rear Motor (3):",Double.toString(RRear.getCurrentPosition()));
            telemetry.addData("Right Rear Motor Target:",Double.toString(distances[3]));
            telemetry.update();
            //check for exit
            if ((Math.abs(LFront.getCurrentPosition()-distances[0])>LFDif+50)){
                exit=true;
            }
            else{
                LFDif=Math.abs(LFront.getCurrentPosition()-distances[0]);
            }
            if ((Math.abs(LRear.getCurrentPosition()-distances[2])>LRDif+50)){
                exit=true;
            }
            else{
                LRDif=Math.abs(LRear.getCurrentPosition()-distances[2]);
            }
            if ((Math.abs(RFront.getCurrentPosition()-distances[1])>RFDif+50)){
                exit=true;
            }
            else{
                RFDif=Math.abs(RFront.getCurrentPosition()-distances[1]);
            }
            if ((Math.abs(RRear.getCurrentPosition()-distances[3])>RRDif+50)){
                exit=true;
            }
            else{
                RRDif=Math.abs(RRear.getCurrentPosition()-distances[3]);
            }
        }
        //stop motors
        LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Get steering correction
     * @param desiredHeading target heading
     * @param proportionalGain rate of change
     * @return the amount to be corrected
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double targetHeading = desiredHeading;  // Save for telemetry
        // Determine the heading current error
        double headingError = targetHeading - getHeading();
        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;
        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * get gyro heading in degrees
     */
    public double getHeading(){
        YawPitchRollAngles orientation=imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * robot movement to wheel movement
     * @param xDistance is x coordinate change
     * @param yDistance is y coordinate change
     * @param turnAngle is angle of turn
     * @return double list of leftFront wheel movement, rightFront wheel movement, leftRear wheel movement, rightRear wheel movement, in encoder ticks
     */
    public static double[] calculateWheelMovement(double xDistance, double yDistance, double turnAngle) {
        //wheel diameter
        double diameter=9.6;
        // Define the wheel circumference
        double wheelCircumference = Math.PI * diameter;
        //Calculate total displacement
        double displacement = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
        // Calculate the average wheel distance
        double averageDistance = displacement / Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
        // Calculate the wheel movement for each direction
        double leftFront = (xDistance + displacement * Math.cos(Math.atan2(yDistance, xDistance))) / wheelCircumference;
        double rightFront = (xDistance - displacement * Math.cos(Math.atan2(yDistance, xDistance))) / wheelCircumference;
        double leftRear = (xDistance + displacement * Math.cos(Math.atan2(-yDistance, xDistance))) / wheelCircumference;
        double rightRear = (xDistance - displacement * Math.cos(Math.atan2(-yDistance, xDistance))) / wheelCircumference;
        // Adjust for turn angle
        if (turnAngle != 0) {
            double turnRadius = displacement / (2 * Math.PI * turnAngle / 360);
            double turnCorrection = (turnRadius - diameter/2) / turnRadius;
            leftFront *= turnCorrection;
            rightFront *= turnCorrection;
            leftRear *= turnCorrection;
            rightRear *= turnCorrection;
        }
        leftFront=movementToEncoderTicks(leftFront);
        rightFront=movementToEncoderTicks(rightFront);
        leftRear=movementToEncoderTicks(leftRear);
        rightRear=movementToEncoderTicks(rightRear);
        return new double[]{leftFront, rightFront, leftRear, rightRear};
    }

    /**
     * Function to calculate motor encoder ticks from rotation distance.
     * @param distance Distance rotated in cm.
     * @return Number of encoder ticks travled.
     */
    public static double movementToEncoderTicks(double distance){
        //Calculate diameter
        double circumference=Math.PI*9.6;
        //Calculate number of rotations
        double rotations=distance/circumference;
        //Calculate ticks per revolution (using formula given by GoBilda)
        double ticksPerRevolution=((((1+(46/17))) * (1+(46/11))) * 28);
        //Return number of encoder ticks
        return rotations*ticksPerRevolution;
    }

    /**
     * wheel movement to robot movement
     * calcuations are in cm.
     */
    public static double[] calculateRobotMovementFromWheel(double leftFront, double rightFront, double leftRear, double rightRear) {
        //wheel diameter
        double diameter=9.6;
        // Define the wheel circumference
        double wheelCircumference = Math.PI * diameter;
        // Calculate the average wheel distance
        double averageDistance = (leftFront + rightFront + leftRear + rightRear) / 4;
        // Calculate the total distance moved
        double totalDistance = (leftFront + leftRear + rightFront + rightRear) / 4;
        // Calculate the x and y distances
        double xDistance = (leftFront - rightFront + leftRear - rightRear) / wheelCircumference;
        double yDistance = (leftFront + rightFront - leftRear - rightRear) / wheelCircumference;
        // Calculate the rotation
        double rotation = (leftFront + rightFront + leftRear + rightRear) / wheelCircumference;
        return new double[]{xDistance, yDistance, rotation};
    }
    /**
     * Wait Function
     */
    //This is only here for the sake of not writing a for loop every time you need to wait.
    public void waitForTime(double seconds){
        //reset runtime
        runtime.reset();
        //wait untill runtime exeeds time limit
        while(runtime.seconds()<=seconds){
            //update telemtry
            telemetry.addData("Time","%4.1f S Elapsed",runtime.seconds());
            //Constantly call the armToPosition function.
            armToPosition(armPosition, wristPosition, intakeSpeed, linearPos);
        }
    }
    
    /**
     * arm function
     */
    public void armToPosition(double armPos, double wristPos, double intakeSpeed, double linearpos){
        //set arm motor target position
        armMotor.setTargetPosition((int)(armPos));
        //set motor velocity
        ((DcMotorEx) armMotor).setVelocity(2100);
        //run arm motor to position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //telemetry if motor exceeded current limit
        if (((DcMotorEx) armMotor).isOverCurrent()){
            telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
        }
        //If the wrist pos passed in is greater than 8.333, then reset it to 8.333.
        //This is to prevent the servo from jaming itself against the arm and burning
        //out due to a bad value being passed in.
        if (wristPos<=8.333){
            //If value is within range
            wrist.setPosition(wristPos);
        }
        else if (wristPos<3.333){
            //If value is out of range in the negative direction
            wrist.setPosition(0);
        }
        else{
            //If value is out of range in the positive direction
            wrist.setPosition(8.333);
        }
        telemetry.addData("Wrist position",wristPos);
        //set intake power/speed
        intake.setPower(intakeSpeed);
    }
    
    /**
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addLine("Path Completed.");
        telemetry.update();
    }

}
