#include "CommandLine.h"


int Commandline::read(std::ostream& fout, ArgumentParser& arguments)
{
	SimOptions::SLAM_MODE mode;
	std::string str;
	SimOptions::NoiseType noiseType;
	double minVal, maxVal;
	int pos = 1;

	while (pos < arguments.argc())
	{
		if (arguments.read(pos, "-mode", (int&)mode))
		{
			_simOptions->setMode(mode);
			fout << "Simulation Mode: " << ((!mode)?"HandHeld":"Stage") << std::endl;
		}
		else if (arguments.read(pos, "-calib", str))
		{
			_simOptions->setCalibFilePath(str);
			fout << "Calib File: " << str << std::endl;
		}
		else if (arguments.read(pos, "-map", str))
		{
			_simOptions->setMapFilePath(str);
			fout << "Map Path: " << str << std::endl;
		}
		else if (arguments.read(pos, "-o", str))
		{
			_simOptions->setOutDir(str);
			fout << "OutPut Dir: " << str << std::endl;
		}
		else if (arguments.read(pos, "-type", (int&)noiseType))
		{
			_simOptions->setNoiseType(noiseType);
			if (noiseType == SimOptions::noise2D)
				fout << "Noise add to 2D points" << std::endl;
			else
				fout << "Noise add to 3D points" << std::endl;
		}
		else if (arguments.read(pos, "--noise", minVal, maxVal ))
		{
			_simOptions->setMaxNoiseVal(maxVal);
			_simOptions->setMinNoiseVal(minVal);

			fout << "Noise val: " << minVal << " ~ "<< maxVal <<std::endl;
		}
		else
			++pos;
	}

	return 0;
}





void Commandline::getUsage(ApplicationUsage& usage)
{
	usage.addCommandLineOption("-mode <int>", "Specify the simulation mode:\nHandHeld_CAM = 0 \nSTAGE_CAM = 1 \n");
	usage.addCommandLineOption("-type <int>", "Specify the noise type:\n2D = 0 \n3D = 1 \n");
	usage.addCommandLineOption("-calib <string>", "Specify the calib file path.");	
	usage.addCommandLineOption("-map <string>", "Specify the map file path.");
	usage.addCommandLineOption("-o <string>", "Specify the output dir.");
	usage.addCommandLineOption("--noise <double double>", "Specify the noise range.");
	
}
