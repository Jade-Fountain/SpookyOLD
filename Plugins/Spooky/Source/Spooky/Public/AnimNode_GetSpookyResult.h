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

#include "Animation/AnimNodeBase.h"
#include "AnimNode_GetSpookyResult.generated.h"

USTRUCT()
struct FAnimNode_GetSpookyResult : public FAnimNode_Base 
{
	
	GENERATED_USTRUCT_BODY()

public:
	FAnimNode_GetSpookyResult();

	// FAnimNode_Base interface
	virtual void Initialize(const FAnimationInitializeContext& Context) override;
	virtual void CacheBones(const FAnimationCacheBonesContext& Context) override;
	virtual void Update(const FAnimationUpdateContext& Context) override;
	virtual void Evaluate(FPoseContext& Output) override;
	virtual void GatherDebugData(FNodeDebugData& DebugData) override;
	// End of FAnimNode_Base interface

#if WITH_EDITOR
	bool InitBoneMap();
	void SetTargetSkeleton(const USkeleton* Skeleton);
	void ValueSet_Name(int32 ChangeAtIndex);
	void ValueSet_ID(int32 ChangeAtIndex);
	void Add_BoneMap(int32 AddAtIndex);
	void Del_BoneMap();
	void Revert_BoneMap();
#endif


};