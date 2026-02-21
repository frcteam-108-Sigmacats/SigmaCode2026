package frc.robot.subsystems.SpinDexer;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SpindexerIOReal implements SpinDexerIO { 
    private SparkMax spinDexer;

    private SparkFlex kicker1;
    private SparkFlex kicker2;

    private SparkMaxConfig configSpinDexer = new SparkMaxConfig();
    private SparkFlexConfig configSparkFlex = new SparkFlexConfig();

    public SpindexerIOReal(){
        spinDexer = new SparkMax(SpindexerConstants.spinDexerID, MotorType.kBrushless);
        kicker1 = new SparkFlex(SpindexerConstants.kicker1ID, MotorType.kBrushless);
        kicker2 = new SparkFlex(SpindexerConstants.kicker2ID, MotorType.kBrushless);
        configSpinDexer.smartCurrentLimit(SpindexerConstants.spinDexerCurrentLimit);
        configSpinDexer.idleMode(IdleMode.kCoast);
        configSparkFlex.smartCurrentLimit(40);
        configSparkFlex.idleMode(IdleMode.kCoast);

        spinDexer.configure(configSpinDexer, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kicker1.configure(configSparkFlex, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kicker2.configure(configSparkFlex, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    @Override
    public void updateInputs(SpinDexerIOInputs inputs){
        inputs.spinDexerDisconnected = spinDexer.getLastError() == REVLibError.kCANDisconnected;
        inputs.spinDexerKickerMotorVoltage = spinDexer.getAppliedOutput();
        inputs.spinDexerMotorCurrent = spinDexer.getOutputCurrent();
        inputs.spinDexerKickerDisconnected = kicker1.getLastError() == REVLibError.kCANDisconnected;
        inputs.spinDexerKickerMotorVoltage = kicker1.getAppliedOutput();
        inputs.spinDexerMotorCurrent = kicker1.getOutputCurrent();
    }}