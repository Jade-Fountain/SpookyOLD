#include "stdafx.h"
#include "CppUnitTest.h"
#include <vector>
#include "../Source/UnrealFusion/Fusion/Utilities/CalibrationUtilities.h"
#include <Windows.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace FusionTesting
{		
	TEST_CLASS(UnitTest1)
	{
	public:
		// Convert a wide Unicode string to an UTF8 string
		std::string utf8_encode(const std::wstring &wstr)
		{
			if (wstr.empty()) return std::string();
			int size_needed = WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), NULL, 0, NULL, NULL);
			std::string strTo(size_needed, 0);
			WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), &strTo[0], size_needed, NULL, NULL);
			return strTo;
		}

		// Convert an UTF8 string to a wide Unicode String
		std::wstring utf8_decode(const std::string &str)
		{
			if (str.empty()) return std::wstring();
			int size_needed = MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), NULL, 0);
			std::wstring wstrTo(size_needed, 0);
			MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), &wstrTo[0], size_needed);
			return wstrTo;
		}

		TEST_METHOD(CalibratePositionalData)
		{
			int N = 1000;
			std::vector<Eigen::Vector3f> samplesA(N);
			std::vector<Eigen::Vector3f> samplesB(N);
			float noise = 0.01;

			Eigen::Transform<float, 3, Eigen::Affine> X = Eigen::Quaternionf::UnitRandom() * Eigen::Translation3f(Eigen::Vector3f::Random());

			for (int i = 0; i < N; i++) {
				samplesA[i] = Eigen::Vector3f::Random();
				samplesB[i] = X * samplesA[i] + Eigen::Vector3f::Random() * 0.01;
			}

			
			Eigen::Transform<float, 3, Eigen::Affine> X_ = fusion::utility::PositionalCalibration::calibrateIdenticalPair(samplesA, samplesB);

			std::stringstream ss;
			ss << "Correct X = \n" << X.matrix() << std::endl;
			ss << "Calibration Result X = \n" << X_.matrix() << std::endl;
			ss << "Max error = " << (X.matrix() - X_.matrix()).maxCoeff() << std::endl;
			std::wstring widestr = utf8_decode(ss.str());

			bool close_enough = X.isApprox(X_,noise);
			Assert::AreEqual(close_enough, true, widestr.c_str());
		}

	};
}