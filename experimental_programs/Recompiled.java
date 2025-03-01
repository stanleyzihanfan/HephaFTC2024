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
import java.util.*;

public class Recompiled {
    
}
//classes

//motor class with all motor information
class Motor{
    protected DcMotor motor_=null;
    protected int velocity=0;
    protected double targetPosition=0;
    protected void setTargetPos(double target){
        this.targetPosition=target;
        this.motor_.setTargetPosition(this.targetPosition);
    }
    protected void setMotorVelocity(int velocity){
        this.velocity=velocity;
        this.motor_.setVelocity(this.velocity);
    }
}
class Imu{
    protected IMU imu=null;
    protected void setIMU(IMU imu){
        this.imu=imu;
    }
    protected void resetYaw(){
        this.imu.resetYaw();
    }
    protected double getHeading(){
        YawPitchRollAngles orientation=this.imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
//drivetrain
class Drivetrain{
    //motor information lists
    protected List<Motor> drivetrain=new ArrayList<Motor>();
    protected IMU imu=null;
    //set drivetrain motors, 0 is left front, 1 is left rear, 2 is right rear, 3 is right front
    protected void setDrivetrain(List<DcMotor> motors){
        for (int i=0;i<4;i++){
            Motor motor=new Motor();
            motor.motor_=motors.get(i);
            drivetrain.add(motor);
        }
    }
}
//main robot class
class Robot{

}