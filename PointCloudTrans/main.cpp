#include "ReadWriter.h"
#include "Stage\PointCloud.h"


int main(int argc, char **argv)
{
	std::string posFilePath = argv[1];
	std::string oriAscFileDir = argv[2];
	

	SLAM::ReadWriter::MatList x3dList;
	SLAM::ReadWriter::MatList normList;
	SLAM::ReadWriter::ScalarList colorList;
	SLAM::ReadWriter::MatMap rotateMap;
	SLAM::ReadWriter::MatMap transMap;

	SLAM::ReadWriter::FilePathMap pathMap;
	SLAM::ReadWriter::getFileListFromDir(oriAscFileDir, pathMap);
	SLAM::ReadWriter::StrList ascPathList, ascFileNameList;

	for (SLAM::ReadWriter::FilePathMap::iterator itr = pathMap.begin(); itr != pathMap.end(); itr++)
	{
		std::string filePath = itr->second;
		int dotIndex = filePath.rfind('.');
		std::string ext = filePath.substr(dotIndex, filePath.length()-dotIndex);
		if (ext != ".asc")
			continue;
		ascPathList.emplace_back(itr->second);
		char filename[10];
		itoa(itr->first, filename, 10);
		ascFileNameList.emplace_back(filename);
	}
		
	if (!SLAM::ReadWriter::loadPosFile(posFilePath, rotateMap, transMap))
	{
		std::cout << "Failed loading pos file: " << posFilePath << std::endl;
		return EXIT_FAILURE;
	}
	
	if(rotateMap.size()!= transMap.size())
	{
		std::cout << "Wrong pos file for unequal count of R,t: " << posFilePath << std::endl;
		return EXIT_FAILURE;
	}

	if (oriAscFileDir.back() != '\\')oriAscFileDir += "\\";
	oriAscFileDir += "trans\\";
	if (access(oriAscFileDir.c_str(), 0) == -1)
	{
		if (_mkdir(oriAscFileDir.c_str()))
		{
			std::cout << "Cannot create matched output dir!" << std::endl;
			return false;
		}
	}
	int posCount = rotateMap.size();
	for (SLAM::ReadWriter::MatMap::iterator posItr= rotateMap.begin(); posItr != rotateMap.end(); posItr++)
	{
		x3dList.clear();
		normList.clear();
		colorList.clear();
		int frameIndex = posItr->first;
		if (!SLAM::ReadWriter::loadAscPtCloudFile(ascPathList[posItr->first], x3dList, normList, colorList))
		{
			std::cout << "Failed loading .asc point cloud file: " << ascPathList[frameIndex] << std::endl;
			continue;
		}

		cv::Mat& R = rotateMap[frameIndex];
		cv::Mat& t = transMap[frameIndex];
		if (!PointCloud::p3DCloudTrans(x3dList, R, t))
		{
			std::cout << "p3DCloud transforming wrong!" << std::endl;
			return EXIT_FAILURE;
		}
		if (!PointCloud::normVectorTrans(normList, R, t))
		{
			std::cout << "normVector transforming wrong!" << std::endl;
			return EXIT_FAILURE;
		}
		std::string savPath = oriAscFileDir + ascFileNameList[frameIndex] + ".asc";
		if(SLAM::ReadWriter::saveAscPtCloudFile(savPath, x3dList, normList, colorList))
		{
			std::cout << "Succeeded saving .asc point cloud file: " << savPath<< std::endl;
		}
		else
		{
			std::cout << "Failed saving .asc point cloud file: " << savPath << std::endl;
			return EXIT_FAILURE;
		}
	}
	return EXIT_SUCCESS;
	
}