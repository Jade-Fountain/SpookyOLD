# Spooky UE4
Skeleton-centered Virtual Reality Sensor Fusion plugin for the Unreal Engine

## Aims and Introduction
The rapidly improving quality and availability of head mounted displays (HMDs) has seen a large increase in public interest in virtual reality (VR). HMDs allow for a wide variety of immersive experiences unavailable to other display platforms. However, HMDs typically block out the real world, leaving the user feeling disembodied in the virtual space. Without tracking of the user’s body and hands, interactions within the environment are limited. Furthermore, without visual body and hand representation within the virtual environment, states of presence and immersion are impeded. If the virtual environment is multi-user, communication can also be impeded by inaccurate or limited body or hand tracking.
 
Tracking systems for achieving high fidelity body and hand tracking are prohibitively expensive. For example, OptiTrack, Vicon, and similar gold-standard motion capture systems can have costs in the range of thousands to hundreds of thousands of US dollars. Many low cost devices exist as alternatives, but often do not provide the required tracking quality and range for many purposes. Some examples include Leap Motion, Microsoft Kinect, Perception Neuron, Oculus Rift, HTC Vive and PlayStation VR. These devices cost from one hundred to a few thousand US dollars, but suffer from issues such as limited tracking volume, occlusion, drift and low accuracy. This repository represents work towards creating highly accessible fusion software which allows for the combination of low cost tracking systems. The objective is to bridge the quality gap between commodity and gold-standard tracking systems. By minimizing the need for user configuration, the software aims to decrease required technical expertise and increase access to high-quality tracking for businesses, research laboratories and hobbyists.

## Dependencies

At this time, only windows is supported

UE4 Plugin Optional Dependencies:
 - [Kinect4Unreal](http://www.opaque.media/kinect-4-unreal/) for using the Microsoft Kinect v2
 - [OptiTrack](http://v110.wiki.optitrack.com/index.php?title=OptiTrack_Unreal_Engine_4_Plugin) for using with an OptiTrack motion capture system
 - VR systems are also supported through UE4 native support VR support (e.g. Oculus Rift w. Touch, HTC Vive, SteamVR, etc.)
 
Each of the above dependencies are optional (following the instructions below).

## How to use

1. Install the relevant plugin dependencies for your planned use as described above.
2. Download and extract the latest release of [Spooky](https://github.com/JakeFountain/Spooky/releases/latest).
3. Edit UnrealFusion.uproject with a text editor and **remove any plugins** that you dont have installed in your UE4 editor.
4. Open UnrealFusion.uproject in the same version of UE4 Editor that you are using for your project. It will prompt you to build - accept and wait for it to build. (If building fails, then try opening UnrealFusion.sln in Microsoft Visual Studio 2015 and build manually with Ctrl+Shift+B. Then reopen UnrealFusion.project.)
> The project should open to Content/Spooky/Maps/SpookySoccer.umap (requires Kinect v2 and VR dependencies installed).
> Ensure SteamVR or Oculus clients are running and the Kinect v2 is connected, then click the dropdown next to the play button and select VR preview.
> Face the Kinect v2 and hold the VR controllers in the correct hands (check thumb polarity). 
> Walk and move your hands to allow spooky to gather data. 
> After a short time your body will appear, and you can kick the soccer ball. 
> Spawn more balls by clicking the touchpad or thumbstick.
> Spooky saves the latest config in \<ProjectName\>/Saved/System1_System2.spky by default, in a human readable format. 
> This location can be changed in the VRGraveyard blueprint at the SpookyFusionPlant->SetSaveDirectory() function call.
> Spooky will automatically detect changes in the position of the Kinect v2 or in the VR tracking space and recalibration will occur after about a minute.
5. To use with your own project:
    * Copy Plugins/Spooky to either your \<ProjectName\>/Plugins folder or Engine/Plugins folder. Go to the Plugins settings and enable Spooky. Restart the editor when prompted.
    * Merge the Content folder with your \<ProjectName\>/Content folder

## Spooky Blueprint Structure

Terms and definitions:

#### Spooky Fusion Plant
Main class for interacting with Spooky.
#### Spirit
A blueprint which sends data from a tracking device to a Spooky Fusion Plant.
#### Skeleton 
A blueprint class which serves as the output of the Spooky Fusion Plant.
#### Graveyard 
A blueprint which links together multiple spirits and a single skeleton to configure the sensor fusion process.
#### Toys
Extra UE4 assets designed for demonstrating Spooky.


