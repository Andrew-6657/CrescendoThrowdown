package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
//import frc.robot.Constants.CodeConstants;
//import frc.robot.Constants.OuttakeConstants;
import org.littletonrobotics.junction.AutoLogOutput;


public class ShooterIO_Sim implements ShooterIO{
    

// Flywheel Motor Controllers
TalonFX followerFlywheel = new TalonFX(Constants.RobotConstants.CANID.kRightFlywheel);
TalonFX leaderFlywheel = new TalonFX(Constants.RobotConstants.CANID.kLeftFlywheel);















    @Override
    public void updateInputs(ShooterIOInputs inputs) {

    }

}
