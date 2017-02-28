#include "UnrealFusion.h"
#include <string>
#pragma once

inline void FUSION_LOG(std::string s){
	FString str(s.c_str());
	UE_LOG(LogTemp, Warning, TEXT("FUSION LOG : %s"),*str);
}