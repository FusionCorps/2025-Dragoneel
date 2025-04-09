# 2025-Dragoneel
6672's codebase for our  robot Mantis, competing in the FRC 2025 [REEFSCAPE](https://www.firstinspires.org/robotics/frc/game-and-season) game.

# Mechanisms:
- Swerve drivetrain with Kraken X60 drive motors, Falcon 500 steer motors on CANivore with Pigeon 2 gyro, SDS MK4n L1+ ratio swerve modules
- Custom-built three-stage custom cascade elevator driven by two Kraken X60s
- Coral + algae-scoring (and algae-intaking) mechanism on a wrist, driven by NEO Vortexes
- Climber driven by a Kraken X60

# Software Features:
- Full field-centric control, with auto-alignment to the reef (for coral scoring and algae pickup)
- Pose localization based on Arducams used with Photonvision
- Can score algae in Net and Processor
- Can score coral at all reef levels
- Capability to score up to four coral at L4 reef height in autonomous period (theoretically)
- Dynamic drivetrain speed limiting based on the current state of the elevator and wrist
- Logging with AdvantageKit, fully articulated robot assets (except climb) in AdvantageScope
- Most useful changeable fields are tunable on-the-fly
