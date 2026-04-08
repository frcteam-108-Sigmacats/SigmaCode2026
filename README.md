# Robot Rewind🔄
[![CI Test Bot](https://github.com/frcteam-108-Sigmacats/SigmaCode2026/actions/workflows/testbottimed.yml/badge.svg)](https://github.com/frcteam-108-Sigmacats/SigmaCode2026/actions/workflows/testbottimed.yml)
[![CI Real Bot](https://github.com/frcteam-108-Sigmacats/SigmaCode2026/actions/workflows/realbot.yml/badge.svg)](https://github.com/frcteam-108-Sigmacats/SigmaCode2026/actions/workflows/realbot.yml)

# Overview
This robot code structure is using the hardware abstraction implementation using the AdvantageKit template so that we would be able to swap between simulation and hardware quickly. Simulation was helpful for getting the general concepts working and being able to log all the inputs we needed helped with easier troubleshooting of the robot. The robot is divided into 4 subsystems consisting of the Drive, Intake, Shooter, and SpinDexer
# Auto
Our Auto utilizes PathPlanner follow to path as well as a custom drive over the bump command to efficiently and quickly go through the bump over auto, grab fuel, and shoot into our hub. We constantly reset our pose on the fly since it does not seem like the limelights are updating our pose when going over the bump
# Subsystem Breakdown
## Drive
The drive consists of 4 KrakenX60s for the drive with 4 Neo550s for the turn and a Pigeon 2.0 for the gyroscope. Using a modified OdometryThread we were able to register both Phoenix and Rev product signals to get fast odometry readings. 

We also have 3 limelights that are inside the Drive Subsystem in which they help grab robot pose estimation. We use a set of conditions to check if the pose estimation given from the limelight's are valid and if so we create the standard deviations for the pose estimator to indicate if the limelight's pose estimation is very strong to rely on or not which is checking tag distance and tag counts
## Intake
The Intake consists of 1 Neo Vortex that is attached to the roller of the intake to help intake fuel. There is no actuation for the intake so we rely on a specific mechanism created to where if we reverse the intake it will unlatch the intake causing it to come out at the start of auto. The intake roller relies on speed percentage and nothing else
## SpinDexer
The SpinDexer consists of 1 Neo 1:1 for the wheel in the middle of the hopper and 2 Neo Vortexes for the kickers that help bring the fuel to the shooter. The SpinDexer spins in a clockwise motion to help better put in fuel into the kickers and not lose any fuel in the process. 

The 2 Kickers are used to move the balls to the shooter to be shot out. All 3 motors rely on speed percentages. 

There is also a CANRange inside the SpinDexer to keep track of how many balls we shoot out to score or pass
## Shooter
The Shooter consists of 1 Neo Vortext for the turret, 1 Through Bore Encoder for absolute tracking of the turret at all times, 2 KrakenX60s for the shooter wheels, and 1 Neo550 for the adjustable hood. 

The Through Bore encoder is used to always keep track of our hood so there is no need for a HOMing setup. The turret relies on the vortex internal encoder for actual feedback for the PID loop and the turret tracks the goal using trigonometry to find the angle that forms between the robot pose and target pose. It also relies on pose estimation to know if the shooter should point at the hub or one of the corners in the alliance zone for passing. 

The adjustable hood and velocity for shooter wheels were put into a Map where we grabbed the data for what the wheels and hood had to be at the distance to be able to make the shot and then used interpolation and distance between the pose of the robot to the target pose to calculate the correct speed the wheels need to be at and the angle that the hood needs to be at to shoot.

Shoot On The Move (SOTM). We use SOTM for our robot where we grab the chassis speeds of the robot, calculate the time of flight of our ball to shoot from our current position, and offset the target pose based on the speed * time formula to correctly get an accurate SOTM. We also take into account the angular velocity of the robot using the turret offset transform so we can SOTM while rotating.
