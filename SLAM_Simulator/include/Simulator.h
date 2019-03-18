#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <opencv2/core/core.hpp>

#include "ReadWriter.h"
#include "System.h"

#include "SimOptions.h"

#include <iostream>
#include <fstream>
#include <io.h>  
#include <iomanip>

#define HISTO_LENGTH 10


class Simulator
{
public:

	Simulator() {}
	~Simulator() {}

	const SimOptions* getSimOptions() { return _simOptions; }
	void setSimOptions(const SimOptions* simOptions) { _simOptions = simOptions; }

	SLAM::GlobalMap& getRealMap() { return _realMap; }
	void setRealMap(SLAM::GlobalMap& map) { _realMap = map; }

	SLAM::Camera& getRealCamera() { return _realCamera; }
	void setRealCamera(SLAM::Camera& cam) { _realCamera = cam; }

	bool loadMapFile();

	bool run();

	bool processSimFrames();

private:
	const SimOptions*													_simOptions;
	SLAM::GlobalMap														_realMap;
	SLAM::Camera														_realCamera;
	std::map<SLAM::Frame*, SLAM::Frame*, SLAM::Frame::FrameComparator>	_simFrameMap;
};



#endif // !SIMULATOR_H