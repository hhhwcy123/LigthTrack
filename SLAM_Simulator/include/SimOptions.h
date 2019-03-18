#ifndef SIMOPTIONS_H
#define SIMOPTIONS_H

#include <ostream>

class  SimOptions
{
public:
	enum SLAM_MODE
	{
		HandHeld,
		Stage
	};

	enum NoiseType
	{
		noise2D,
		noise3D
	};
	SimOptions():
		_mode(HandHeld),
		_noiseType(noise2D),
		_minNoise(0),
		_maxNoise(0)
	{}

	~SimOptions() {}

	SimOptions& operator = (const SimOptions& rhs)
	{
		if (this == &rhs) return *this;
		return *this;
	}

	void setMode(const SLAM_MODE& dir) { _mode = dir; }
	const SLAM_MODE& getMode() const { return _mode; }

	void setCalibFilePath(std::string path) { _calibPath = path; }
	const std::string& getCalibFilePath() const { return _calibPath; }

	void setMapFilePath(std::string path) { _mapFilePath = path; }
	const std::string& getMapFilePath() const { return _mapFilePath; }

	void setOutDir(std::string dir) { _outDir = dir; }
	const std::string& getOutDir() const { return _outDir; }

	void setNoiseType(const NoiseType& type) { _noiseType = type; }
	const NoiseType& getNoiseType() const { return _noiseType; }

	void setMaxNoiseVal(const double& val) { _maxNoise = val; }
	const double& getMaxNoiseVal() const { return _maxNoise; }

	void setMinNoiseVal(const double& val) { _minNoise = val; }
	const double& getMinNoiseVal() const { return _minNoise; }

private:

	SLAM_MODE							_mode;
	std::string							_calibPath;
	std::string							_mapFilePath;
	std::string                         _outDir;
	NoiseType							_noiseType;
	double							    _minNoise;
	double								_maxNoise;
	

};

#endif //!SIMOPTIONS_H