#include "stdafx.h"
#include "CppUnitTest.h"
#include <vector>
#include "../Source/UnrealFusion/Fusion/Utilities/CalibrationUtilities.h"
#include "../Source/UnrealFusion/Fusion/Utilities/CommonMath.h"
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
			float noise = 0.01f;

			Eigen::Transform<float, 3, Eigen::Affine> X = Eigen::Quaternionf::UnitRandom() * Eigen::Translation3f(Eigen::Vector3f::Random());

			for (int i = 0; i < N; i++) {
				samplesA[i] = Eigen::Vector3f::Random();
				samplesB[i] = X * samplesA[i] + Eigen::Vector3f::Random() * 0.01;
			}

			float error = 0;
			Eigen::Transform<float, 3, Eigen::Affine> X_ = fusion::utility::calibration::Position::calibrateIdenticalPairTransform(samplesA, samplesB, &error);

			std::stringstream ss;
			ss << "Correct X = \n" << X.matrix() << std::endl;
			ss << "Calibration Result X = \n" << X_.matrix() << std::endl;
			ss << "Error = " << error << std::endl;
			std::wstring widestr = utf8_decode(ss.str());

			bool close_enough = X.isApprox(X_,noise);
			Assert::AreEqual(close_enough, true, widestr.c_str());
		}

		TEST_METHOD(GetCircleNormal) 
		{
			Eigen::Vector3f A(1,0,0);
			Eigen::Vector3f B(0,1,0);
			Eigen::Vector3f C(0,0,1);

			fusion::utility::Line n = fusion::utility::getCircleNormal(A,B,C);

			Eigen::Vector3f dir = n.direction;
			bool success = dir.cross(Eigen::Vector3f(1,1,1)).isApprox(Eigen::Vector3f(0,0,0),0.0001);
			success = success && n.origin.isApprox(Eigen::Vector3f(1,1,1) * 1/3,0.0001);

			Assert::AreEqual(success, true);
		}

		TEST_METHOD(GetSphere) {
			//Basic sphere
			Eigen::Vector3f A(1, 1, 0);
			Eigen::Vector3f B(0, 1, 1);
			Eigen::Vector3f C(1, 0, 1);
			Eigen::Vector3f D(2, 1, 1);

			fusion::utility::Sphere s = fusion::utility::getSphereFrom4Points(A,B,C,D);

			bool success = s.center.isApprox(Eigen::Vector3f(1,1,1)) && std::fabs(s.r-1) < 0.0001;
			Assert::AreEqual(success, true);

			//Random spheres
			for(int j = 0; j < 100 ; j++){
				std::vector<Eigen::Vector3f> points(4);
				for (int i = 0; i < 4; i++) {
					points[i] = Eigen::Vector3f::Random();
				}
				fusion::utility::Sphere s = fusion::utility::getSphereFrom4Points(points[0], points[1], points[2], points[3]);
				Eigen::Vector4f errors(4);
				for (int i = 0; i < 4; i++) {
					errors(i) = (points[i] - s.center).norm() - s.r;
				}
				success = errors.norm() < 0.0001;


				std::stringstream ss;
				ss << "Points = \n" << points[0] << "," << points[1] << "," << points[2] << "," << points[3] << std::endl;
				ss << "Sphere = \n" << s.center << "," << s.r << std::endl;
				ss << "Errors = " << errors << std::endl;
				std::wstring widestr = utf8_decode(ss.str());

				Assert::AreEqual(success,true, widestr.c_str());
			}

		}


	};
}