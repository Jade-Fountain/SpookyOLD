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
#include "Spooky.h"
#include "Logging.h"
#include "AnimNode_GetSpookyResult.h"

	FAnimNode_GetSpookyResult::FAnimNode_GetSpookyResult(){

	}
	// FAnimNode_Base interface
	void FAnimNode_GetSpookyResult::Initialize(const FAnimationInitializeContext& Context) {

	}
	void FAnimNode_GetSpookyResult::CacheBones(const FAnimationCacheBonesContext& Context) {

	}
	void FAnimNode_GetSpookyResult::Update(const FAnimationUpdateContext& Context) {
		FUSION_LOG("Updating Spooky Result");
		FUSION_LOG((SpookyFusionPlant!=NULL ? "Valid" : "INVALID"));
	}
	void FAnimNode_GetSpookyResult::Evaluate(FPoseContext& Output) {

	}
	void FAnimNode_GetSpookyResult::GatherDebugData(FNodeDebugData& DebugData) {

	}
	// End of FAnimNode_Base interface

// #if WITH_EDITOR
// 	bool FAnimNode_GetSpookyResult::InitBoneMap(){

// 	}
// 	void FAnimNode_GetSpookyResult::SetTargetSkeleton(const USkeleton* Skeleton){

// 	}
// 	void FAnimNode_GetSpookyResult::ValueSet_Name(int32 ChangeAtIndex){

// 	}
// 	void FAnimNode_GetSpookyResult::ValueSet_ID(int32 ChangeAtIndex){

// 	}
// 	void FAnimNode_GetSpookyResult::Add_BoneMap(int32 AddAtIndex){

// 	}
// 	void FAnimNode_GetSpookyResult::Del_BoneMap(){

// 	}
// 	void FAnimNode_GetSpookyResult::Revert_BoneMap(){

// 	}
// #endif
