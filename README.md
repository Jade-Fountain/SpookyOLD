# Spooky UE4
WIP Skeleton-centered Virtual Reality Sensor Fusion plugin for the Unreal Engine

## Aims and Introduction
The rapidly improving quality and availability of head mounted displays (HMDs) has seen a large increase in public interest in virtual reality (VR). HMDs allow for a wide variety of immersive experiences unavailable to other display platforms. However, HMDs typically block out the real world, leaving the user feeling disembodied in the virtual space. Without tracking of the user’s body and hands, interactions within the environment are limited. Furthermore, without visual body and hand representation within the virtual environment, states of presence and immersion are impeded. If the virtual environment is multi-user, communication can also be impeded by inaccurate or limited body or hand tracking.
 
Tracking systems for achieving high fidelity body and hand tracking are prohibitively expensive. For example, OptiTrack, Vicon, and similar gold-standard motion capture systems can have costs in the range of thousands to hundreds of thousands of US dollars. Many low cost devices exist as alternatives, but often do not provide the required tracking quality and range for many purposes. Some examples include Leap Motion, Microsoft Kinect, Perception Neuron, Oculus Rift, HTC Vive and PlayStation VR. These devices cost from one hundred to a few thousand US dollars, but suffer from issues such as limited tracking volume, occlusion, drift and low accuracy. This repository represents work towards creating highly accessible fusion software which allows for the combination of low cost tracking systems. The objective is to bridge the quality gap between commodity and gold-standard tracking systems. By minimizing the need for user configuration, the software aims to decrease required technical expertise and increase access to high-quality tracking for businesses, research laboratories and hobbyists.

## Dependencies

###Only supports Windows

UE4 Plugin Optional Dependencies:
 - [Kinect4Unreal](http://www.opaque.media/kinect-4-unreal/) for using the Microsoft Kinect v2
 - [OptiTrack](http://v110.wiki.optitrack.com/index.php?title=OptiTrack_Unreal_Engine_4_Plugin) for using with an OptiTrack motion capture system
 
Each of the above dependencies are optional, but are supported by Spooky with no additional blueprint configuration.

## How to use

1. Install the relevant plugin dependencies for your planned use as described above.
2. Download and extract the latest release of [Spooky](https://github.com/JakeFountain/Spooky/releases/latest).
3. Copy Plugins/Spooky to either your <ProjectName>/Plugins folder or Engine/Plugins folder.
4. Merge the Content folder with your <ProjectName>/Content folder.
5. If you have a Kinect v2, start UE4 Editor, and try Content/Spooky/Maps/SpookySoccer.

## Spooky Blueprint Structure

###Terms and definitions:

#### Spooky Fusion Plant - main class for interacting with Spooky
#### Spirit - a blueprint which sends data from a tracking device to a Spooky Fusion Plant
#### Skeleton - a blueprint class which serves as the output of the Spooky Fusion Plant
#### Graveyard - a blueprint which links together multiple spirits and a single skeleton to configure the sensor fusion process
#### Toys - Extra UE4 assets designed for demonstrating Spooky


