/*  This file is part of UnrealFusion, a sensor fusion plugin for VR in the Unreal Engine
    Copyright (C) 2017 Jake Fountain
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include "Components/ActorComponent.h"
#include "Components/PoseableMeshComponent.h"

#include "Fusion/Core.h"
#include "Fusion/FusionTypes.h"

#include <iostream>
#include <vector>
#include <string>

//Must be last include
#include "FusionPlant.generated.h"

//Unreal engine specific struct containing the results of a calibration between two 3D sensor systems
USTRUCT()
struct FCalibrationResult {
	GENERATED_BODY()
		
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Fusion") FTransform transform;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Fusion") bool calibrated = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Fusion") float quality = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Fusion") FString system1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Fusion") FString system2;
};

//Unreal engine interface layer linking to generic code from the fusion module
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class UNREALFUSION_API UFusionPlant : public UActorComponent
{
	GENERATED_BODY()

	//Fusionplant
	static fusion::Core plant;

	//Input Skeletons
	std::vector<UPoseableMeshComponent*> skeletons;
	std::vector<Eigen::Matrix<float, 7, 1>> skeletonCovariances;

	//Configuration
	struct {
		struct {
			float input_m = 1;
			float output_m = 1;
		} units;
	} config;
protected:
	//Fusion result
	UPROPERTY(BlueprintReadOnly)
	UPoseableMeshComponent* fusedSkeleton;

public:	

	// Sets default values for this component's properties
	UFusionPlant();

	// Called when the game starts
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void TickComponent( float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction ) override;

//===========================
//Setup and initialisation
//===========================

	//Add complete skeleton to list of fusion objects
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void Configure(float input_units_m = 1, float output_units_m = 1);


	//Add complete skeleton to list of fusion objects
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void AddSkeleton(UPoseableMeshComponent* poseable_mesh, FVector position_var, FVector4 quaternion_var);

	//Set the output target which will have the complete fused skeleton pose applied
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void SetOutputTarget(UPoseableMeshComponent* poseable_mesh);
	
	//Perform some setup postprocessing
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void FinaliseSetup();

//TODO: Contruction of sensor nodes

	////Add a new sensor node model
	//UFUNCTION(BlueprintCallable, Category = "Fusion")
	//void AddSensorNode(FString nodeName, FTransform initialState, FTransform initialCovariance);

	////Add a new sensor node model
	//UFUNCTION(BlueprintCallable, Category = "Fusion")
	//void SetHomeCoordinateSpace(FString systemName);

//===========================
//Update functions
//===========================
	//Add vec3 measurement
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void AddPositionMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FVector measurement, FVector covariance, bool globalSpace = true, float confidence = 1);
	
	//Add rotation quaternion method
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void AddRotationMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FRotator measurement, FVector4 covariance, bool globalSpace = true, float confidence = 1);

	//Add transform measurement in local space
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void AddPoseMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FTransform measurement, FVector position_var, FVector4 quaternion_var, bool globalSpace = true, float confidence = 1);

	//Adds measurements for whole skeleton
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void addSkeletonMeasurement(int skel_index, float timestamp_sec);

	//Align, calibrate and fuse all added data
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void Fuse(float timestamp_sec);
	
	//Copies the results of fusion to the target skeleton
	// Default output units is centimeters as that is what Unreal Engine uses
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void UpdateSkeletonOutput();
//===========================
//Data retrieval functions
//===========================
	//Gets the calibration result mapping T:s1->s2
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	FCalibrationResult getCalibrationResult(FString s1, FString s2);
	
	//Gets the name of the node which the specified sensor is attached to
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	FString getCorrelationResult(FString s1, int sensorID);

	//Gets the result of fusion for node 
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	FTransform getNodeGlobalPose(FString node);



//===========================
//Utility
//===========================
	//Method to copy data from one poseable mesh to another
	void CopyPose(UPoseableMeshComponent* target, const UPoseableMeshComponent* input);

	//Methods for creating measurements which can then be sent to the fusion plant
	fusion::Measurement::Ptr CreatePositionMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector position, FVector uncertainty, float confidence = 1);
	fusion::Measurement::Ptr CreateRotationMeasurement(FString system_name, int sensorID, float timestamp_sec, FQuat rotation, FVector4 uncertainty, float confidence = 1);
	fusion::Measurement::Ptr CreateScaleMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector scale, FVector uncertainty, float confidence = 1);
	fusion::Measurement::Ptr CreatePoseMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector v, FQuat q, Eigen::Matrix<float,7,1> uncertainty, float confidence = 1);
	
	//Sets data common to all types of measurements
	void SetCommonMeasurementData(fusion::Measurement::Ptr& m, FString system_name, int sensorID, float timestamp_sec, float confidence);

	//Convert names to nodeDescriptors
	std::vector<fusion::NodeDescriptor> convertToNodeDescriptors(const TArray<FString>& names);

	//Convert Transform3D to FMatrix
	FMatrix convert(const fusion::Transform3D& T);
	fusion::Transform3D convert(const FMatrix& T);
//===========================
//DEBUG
//===========================

	//For testing blueprints: TODO delete
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	FVector4 GetTestPosition();

	
};
