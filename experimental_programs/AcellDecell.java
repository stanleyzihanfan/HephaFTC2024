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
import java.lang.Thread;

@Autonomous

public class AcellDecell extends OpMode
{
    //arm servo+motor declaration
    public DcMotor  armMotor    = null; //the arm motor
    public CRServo  intake      = null; //the active intake servo
    public Servo    wrist       = null; //the wrist servo
    // drivetrain wheel motor declaration
    private DcMotorEx LFront=null;
    private DcMotorEx LRear=null;
    private DcMotorEx RFront=null;
    private DcMotorEx RRear=null;
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
        LFront   = hardwareMap.get(DcMotorEx.class, "wheel_0");
        LRear    = hardwareMap.get(DcMotorEx.class, "wheel_1");
        RFront   = hardwareMap.get(DcMotorEx.class, "wheel_3");
        RRear    = hardwareMap.get(DcMotorEx.class, "wheel_2");
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
        //reset runtime
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
        drivegyro(0,10,10,-1,0.01,10,1);
        //forward 17.26 in.
        //right 13.75 in.
        //diag right 13.75 in.
        //diag forward 15.76 in.
        // drivegyro(0, -3000, -1000, 0.1, 0.01, 1000);
    }

    /**
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        //Constantly set the arm positions, as running motors with RUN_TO_POSITION requires this to operate properly.
        armToPosition(armPosition, wristPosition, intakeSpeed,linearPos);
        telemetry.addData("Left Front Motor (0):",Double.toString(LFront.getCurrentPosition()));
        telemetry.addData("Right Front Motor (3):",Double.toString(RFront.getCurrentPosition()));
        telemetry.addData("Left Rear Motor (2):",Double.toString(LRear.getCurrentPosition()));
        telemetry.addData("Right Rear Motor (1):",Double.toString(RRear.getCurrentPosition()));
        telemetry.update();
    }
    
    public void rotate(double twist,double speed){
        double targetdirection=getHeading()+twist;
        if (Math.abs(targetdirection)>180){
            if (targetdirection<0){
                targetdirection=360+targetdirection;
            }
            else{
                targetdirection=360-targetdirection;
            }
        }
        boolean target=(getHeading()>targetdirection);
        while ((getHeading()>targetdirection)==target){
            if (target){
                LFront.setPower(speed);
                LRear.setPower(speed);
                RFront.setPower(-speed);
                RRear.setPower(-speed);
                LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else{
                LFront.setPower(-speed);
                LRear.setPower(-speed);
                RFront.setPower(speed);
                RRear.setPower(speed);
                LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        LFront.setTargetPosition(LFront.getCurrentPosition());
        LRear.setTargetPosition(LRear.getCurrentPosition());
        RFront.setTargetPosition(RFront.getCurrentPosition());
        RRear.setTargetPosition(RRear.getCurrentPosition());
        //stop motors
        LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * main gryo drive function.
     * All distance parameters are cm.
     * Give ~±0.5 cm tolerance(especialy for drive).
     * Twist is in degrees.
     * @param twist is degrees to turn
     * @param strafe is how far sideways
     * @param drive is how far forwards
     * @param maxspeed is max drive speed(in encoder ticks/second^2)(set to -1 to ignore)
     * @param rotationspeed is rotation speed
     * @param time is the time in seconds the robot moves for(not including acceleration/deceleration and normalization max speed)
     * @param acceleration is the acceleration/deceleration speed of the robot in encoder ticks/second^2
     * @return none
     */
    public void drivegyro(double twist, double strafe, double drive, double maxspeed, double rotationspeed, double time, double acceleration){
        //get target heading
        double targetdirection=getHeading()+twist;
        //change distance to movement units
        double newdrive=movementToDistanceUnits(drive, strafe)[0];
        double newstrafe=movementToDistanceUnits(drive, strafe)[1];
        //robot moves to the right when strafe is negative, left when positive
        double[] distances=calculateWheelMovement(newdrive, newstrafe * -1);
        for (int i=0;i<=3;i++){
            if (Math.abs(distances[i])<=5){
                distances[i]=0;
            }
        }
        //caculate speeds
        double[] speeds={Math.abs(distances[0]/time),Math.abs(distances[1]/time),Math.abs(distances[2]/time),Math.abs(distances[3]/time)};
        //get max speed
        double max=speeds[0];
        for (int i=0;i<speeds.length;i++){
            if (speeds[i]>max) max=speeds[i];
        }
        //normalize speeds if it is higher than max
        if (max>maxspeed && maxspeed!=-1){
            for (int i=0;i<speeds.length;i++) speeds[i]=speeds[i]/max*maxspeed;
        }
        //Calculate target wheel encoder position
        distances[0]=LFront.getCurrentPosition()+distances[0];
        distances[1]=RFront.getCurrentPosition()+distances[1];
        distances[2]=LRear.getCurrentPosition()+distances[2];
        distances[3]=RRear.getCurrentPosition()+distances[3];
        //list of accel/decel distances
        double[] accel_distances={0,0,0,0,0,0,0,0};
        //list of time
        double[] times={0,0,0,0,0,0,0,0};
        for (int i=0;i<4;i++){
            //calculate acceleration and/or deceleration distance
            double acel_distance=Math.pow(maxspeed,2)/(2*acceleration);
            double hold_distance=0;
            //hold speed distance
            if (i==0){
                //Left Front wheel
                hold_distance=(Math.abs(distances[0]-LFront.getCurrentPosition()))-2*acel_distance;
            }
            else if (i==1){
                //Right Front wheel
                hold_distance=(Math.abs(distances[1]-RFront.getCurrentPosition()))-2*acel_distance;
            }
            else if (i==2){
                //Left Rear wheel
                hold_distance=(Math.abs(distances[2]-LRear.getCurrentPosition()))-2*acel_distance;
            }
            else{
                //Right Rear wheel
                hold_distance=(Math.abs(distances[3]-RRear.getCurrentPosition()))-2*acel_distance;
            }
            //accel time
            double acel_real_time=maxspeed/acceleration;
            //hold time
            double hold_real_time=hold_distance/maxspeed;
            //add to list
            accel_distances[i]=acel_distance;
            accel_distances[i+4]=hold_distance;
            times[i]=acel_real_time;
            times[i+4]=hold_real_time;
        }
        //wheel one
        LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFront.setTargetPosition((int)distances[0]);
        //wheel two
        LRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRear.setTargetPosition((int)distances[2]);
        //wheel three
        RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFront.setTargetPosition((int)distances[1]);
        //wheel four
        RRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRear.setTargetPosition((int)distances[3]);
        //run motors
        LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //boolean values to show if the motor is ahead or behind the target position
        boolean LF=(LFront.getCurrentPosition()>distances[0]);
        boolean LR=(LRear.getCurrentPosition()>distances[2]);
        boolean RF=(RFront.getCurrentPosition()>distances[1]);
        boolean RR=(RRear.getCurrentPosition()>distances[3]);
        //run stage(0=accel,1=hold,2=decel)
        int[] stage={0,0,0,0};
        //reset runtime
        runtime.reset();
        while (true){
            //call armToPosition to move arm motor+servos
            armToPosition(armPosition, wristPosition, intakeSpeed,linearPos);
            //get steering correction
            double steeringCorrection=getSteeringCorrection(targetdirection, rotationspeed);
            for (int i=0;i<4;i++){
                //change stage
                if (runtime.seconds()>=times[i]){
                    stage[i]=1;
                }
                else if (runtime.seconds()>=times[i]+times[i+4]){
                    stage[i]=2;
                }
                //change speeds list based on stage
                if (stage[i]==0){
                    //x=the current speed(based on time)
                    //x=runtime*final speed/total time
                    //acceleration
                    speeds[i]=(runtime.seconds()*(accel_distances[i]/times[i]))/times[i];
                }
                else if (stage[i]==1){
                    //hold
                    speeds[i]=accel_distances[i+4]/times[i+4];
                }
                else{
                    //x=the current speed(based on time)
                    //x=time remaining*intial speed/total time
                    //deceleration
                    speeds[i]=((times[i]-runtime.seconds())*(accel_distances[i+4]/times[i+4]))/times[i];
                }
            }
            //set motor power(setting it to negative if the wheel is going backwards, and 0 if it isn't moving)
            if (distances[1] < RFront.getCurrentPosition() ) { 
                RFront.setVelocity((speeds[1] * -1) +steeringCorrection);
            }
            else if (distances[1] == RFront.getCurrentPosition()) {
                RFront.setVelocity(0);
            }
            else {
                RFront.setVelocity(speeds[1] + steeringCorrection);
            }
            if (distances[3] < RRear.getCurrentPosition() ) { 
                RRear.setVelocity((speeds[3] * -1) +steeringCorrection);
            }
            else if (distances[3] == RRear.getCurrentPosition()) {
                RRear.setVelocity(0);
            }
            else {
                RRear.setVelocity(speeds[3] + steeringCorrection);
            }
            
            if (distances[0] < LFront.getCurrentPosition() ) { 
                LFront.setVelocity((speeds[0] * -1) - steeringCorrection);
            }
            else if (distances[0] == LFront.getCurrentPosition()) {
                LFront.setVelocity(0);
            }
            else {
                LFront.setVelocity(speeds[0] - steeringCorrection);
            }
            
            if (distances[2] < LRear.getCurrentPosition() ) { 
                LRear.setVelocity((speeds[2] * -1) - steeringCorrection);
            }
            else if (distances[2] == LRear.getCurrentPosition()) {
                LRear.setVelocity(0);
            }
            else {
                LRear.setVelocity(speeds[2] - steeringCorrection);
            }
        
            //telemetry
            telemetry.addData("Left Front Motor (0):",Double.toString(LFront.getCurrentPosition()));
            telemetry.addData("Left Front Motor Target:",Double.toString(distances[0]));
            telemetry.addData("Left Front Motor Speed:",Double.toString(speeds[0]));
            telemetry.addData("Right Front Motor (1):",Double.toString(RFront.getCurrentPosition()));
            telemetry.addData("Right Front Motor Target:",Double.toString(distances[1]));
            telemetry.addData("Right Front Motor Speed:",Double.toString(distances[1]));
            telemetry.addData("Left Rear Motor (2):",Double.toString(LRear.getCurrentPosition()));
            telemetry.addData("Left Rear Motor Target:",Double.toString(distances[2]));
            telemetry.addData("Left Rear Motor Speed:",Double.toString(distances[2]));
            telemetry.addData("Right Rear Motor (3):",Double.toString(RRear.getCurrentPosition()));
            telemetry.addData("Right Rear Motor Target:",Double.toString(distances[3]));
            telemetry.addData("Right Rear Motor Speed:",Double.toString(distances[3]));
            telemetry.update();
            //check for exit
            if (!((LFront.getCurrentPosition()>distances[0])==LF) && !((LRear.getCurrentPosition()>distances[2])==LR) && !((RFront.getCurrentPosition()>distances[1])==RF) && !((RRear.getCurrentPosition()>distances[3])==RR)){
                LFront.setVelocity(0);
                LRear.setVelocity(0);
                RFront.setVelocity(0);
                RRear.setVelocity(0);
                break;
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
        // Multiply the error by the gain to determine the required steering correction. Limit the result to +/- 1.0
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
    public static double[] calculateWheelMovement(double xDistance, double yDistance) {
        //Calculate target angle
        double angle = Math.atan2(yDistance, xDistance);
        //Calculate total displacement
        double displacement = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
        // Calculate the wheel movement for each direction
        double leftFront = displacement*Math.cos(angle+Math.PI/4);
        double rightFront = displacement*Math.cos(angle-Math.PI/4);
        double leftRear = displacement*Math.cos(angle-Math.PI/4);
        double rightRear = displacement*Math.cos(angle+Math.PI/4);
        return new double[]{leftFront, rightFront, leftRear, rightRear};
    }

    /**
     * Function to calculate distance units from rotation distance.
     * @param xdistance Distance moved forwards in cm.
     * @param ydistance Distance moved right in cm.
     * @return Double list of movement units traveled.
     */
    public static double[] movementToDistanceUnits(double xdistance, double ydistance){
        //forward 17.26 in., 43.8404 cm. --> 0.0438404cm/MU
        //right 13.75 in., 34.925 cm. --> 0.034925cm/MU
        //diag right 13.75 in., 34.925 cm. --> 0.034925/MU
        //diag forward 15.76 in., 40.0304 cm. --> 0.0400304cm/MU
        //average 0.0419372cm/MU forward
        double finalY=ydistance/0.034925;
        double finalX=xdistance/0.0419372;
        double[] ans={finalX,finalY};
        return ans;
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
        //set intake power/speed
        intake.setPower(intakeSpeed);
        //set arm motor target position
        armMotor.setTargetPosition((int)(armPos));
        //set motor velocity
        ((DcMotorEx) armMotor).setVelocity(2750);
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
        //set linear slide position
        if (linearpos<100){
            linearpos=100;
        }
        else{
            linearpos=linearpos;
        }
        telemetry.addData("linear Target:",linearpos);
        linearR.setTargetPosition((int) (linearpos));
        linearL.setTargetPosition((int) (linearpos));
        ((DcMotorEx) linearR).setVelocity(1000);
        ((DcMotorEx) linearL).setVelocity(1000);
        linearR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
