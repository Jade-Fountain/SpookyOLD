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
	fusion::Core plant;

	//Input Skeletons
	std::vector<UPoseableMeshComponent*> skeletons;
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
	void AddSkeleton(UPoseableMeshComponent* poseable_mesh);
	
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
	void AddPositionMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FVector measurement, FVector covariance, float confidence = 1);
	
	//Add rotation quaternion method
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void AddRotationMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FQuat measurement, FVector covariance, float confidence = 1);

	//Align, calibrate and fuse all added data
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void Fuse();

//===========================
//Data retrieval functions
//===========================
	//Gets the calibration result mapping T:s1->s2
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	FCalibrationResult getCalibrationResult(FString s1, FString s2);
	
	//Gets the name of the node which the specified sensor is attached to
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	FString getCorrelationResult(FString s1, int sensorID);

//===========================
//Utility
//===========================
	//Method to copy data from one poseable mesh to another
	void CopyPose(UPoseableMeshComponent* target, const UPoseableMeshComponent* input);

	//Methods for creating measurements which can then be sent to the fusion plant
	fusion::Measurement::Ptr CreatePositionMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector position, FVector uncertainty, float confidence = 1);
	fusion::Measurement::Ptr CreateRotationMeasurement(FString system_name, int sensorID, float timestamp_sec, FQuat rotation, FVector uncertainty, float confidence = 1);
	fusion::Measurement::Ptr CreateScaleMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector scale, FVector uncertainty, float confidence = 1);
	fusion::Measurement::Ptr CreateRigidBodyMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector state, FVector uncertainty, float confidence = 1);
	
	//Sets data common to all types of measurements
	void SetCommonMeasurementData(fusion::Measurement::Ptr& m, FString system_name, int sensorID, float timestamp_sec, float confidence);

	//Convert names to nodeDescriptors
	std::vector<fusion::NodeDescriptor> convertToNodeDescriptors(const TArray<FString>& names);
//===========================
//DEBUG
//===========================

	//For testing blueprints: TODO delete
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	FVector4 GetTestPosition();

	
};
