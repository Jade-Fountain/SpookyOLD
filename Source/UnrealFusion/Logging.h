#include "UnrealFusion.h"
#include <string>
#pragma once

inline void LOG_WARNING(std::string s){
	UE_LOG(LogTemp, Warning, TEXT("FUSION LOG : %s"),s.c_str());
}