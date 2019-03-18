#include "ArgumentParser.h"
#include "CommandLine.h"

#include "System.h"
#include "ReadWriter.h"

#include "Simulator.h"


int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		std::cout << "Lack for parameter of the image source path" << std::endl;
		std::cout << "Use Command -h or --help to Display help information" << std::endl;
		return EXIT_SUCCESS;
	}

	////////////从命令行获取参配//////////

	ArgumentParser arguments(&argc, argv);
	arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName() + " [options] parameter ...");
	arguments.getApplicationUsage()->addCommandLineOption("-h or --help", "Display this information");

	Commandline commandline;

	commandline.getUsage(*arguments.getApplicationUsage());

	if (arguments.read("-h") || arguments.read("--help"))
	{
		arguments.getApplicationUsage()->write(std::cout, ApplicationUsage::COMMAND_LINE_OPTION);
		return EXIT_SUCCESS;
	}
	int result = commandline.read(std::cout, arguments);
	if (result) return result;

	const SimOptions* simOptions = commandline.getSimOption();
	const SimOptions::SLAM_MODE& mode = simOptions->getMode();
	switch (mode)
	{
		case SimOptions::HandHeld:
			SLAM::System::instance()->loadHandHeldCamera(simOptions->getCalibFilePath());
			break;
		case SimOptions::Stage:
			SLAM::System::instance()->loadStageCamera(simOptions->getCalibFilePath());
			break;
		default:
			std::cout << "Faild: Wrong input mode param!" << std::endl;
			return EXIT_FAILURE;
	}
	

	Simulator simulator;
	simulator.setSimOptions(simOptions);
	simulator.setRealCamera(SLAM::System::instance()->getCamera());
	
	std::cout << "Loading map file..."<< std::endl;
	if (!simulator.loadMapFile())
	{
		std::cout << "Error! Failed loading map from file:" << simOptions->getMapFilePath() << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "Producing simulating frames..." << std::endl;
	if(!simulator.processSimFrames())
	{
		std::cout << "Error! Failed creating simulating frames:" << simOptions->getMapFilePath() << std::endl;
		return EXIT_FAILURE;
	}
	

	std::cout << "Simulation running..." << std::endl;
	if (!simulator.run())
	{
		std::cout << "Simulation Running Error!" << std::endl;
		return EXIT_FAILURE;
	}
	

}
