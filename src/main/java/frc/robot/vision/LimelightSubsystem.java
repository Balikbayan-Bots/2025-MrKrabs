package frc.robot.vision;

public interface LimelightSubsystem {

  double alignTx();

  double alignTy();

  double alignRz();

  double getTx();

  double getTy();

  void setPipeline(int val);
}
