#ifndef READWRITER_H
#define READWRITER_H

#include <iostream>
#include <fstream>

#include "GlobalMap.h"

#include <io.h>  
#include <iomanip>


#ifdef SLAM_SHARED_LIBS
#ifdef SLAM_IO_EXPORTS
#define READWRITER_API  __declspec(dllexport)
#else
#define READWRITER_API  __declspec(dllimport)
#endif
#else
#define READWRITER_API
#endif

namespace SLAM
{

//class READWRITER_API ReadWriter
class READWRITER_API ReadWriter
{
public:
	typedef std::map<int,cv::Mat> MatMap;
	typedef std::vector<cv::Mat> MatList;
	typedef std::vector<cv::Scalar> ScalarList;

public:
	
	struct Comparator
	{
		bool operator()(std::string str1, std::string str2) const { return str1 < str2; }
	};
	typedef std::map<std::string,std::string, Comparator> FilePathStrMap;
	typedef std::map<int, std::string> FilePathMap;
	typedef std::vector<std::string> StrList;

public:

	static bool readLine(FILE* infile, char* buffer,const int& len);

	static bool saveMapFile(std::string filePath, GlobalMap& globalMap);

	static bool loadMapFile(const std::string filePath, System& system);

	static bool loadFrameWithP3DFile(const std::string filePath, SLAM::FrameList& frameList);

	static bool loadPosFile(const std::string filePath, MatMap& rotationList, MatMap& transList);

	static bool loadPoint3DFile(const std::string filePath, std::vector<cv::Point3d>& p3DList);

	static bool loadAscPtCloudFile(const std::string filePath, MatList& x3DList, MatList& normList, ScalarList& colorList);

	static bool saveAscPtCloudFile(const std::string filePath, const MatList& x3DList, const MatList& normList, const ScalarList& colorList);

	static bool getFileListFromDir(const std::string& dir, FilePathMap& filePathList);

	static bool getFileListFromDir(const std::string& dir, FilePathStrMap& filePathList);

	static int getDirListFromDir(std::string& dir, FilePathMap& dirList);

	static bool loadImageList(std::string imgDir, StrList& leftImgPathList, StrList& rightImgPathList);

	static bool loadHandHeldImageList(std::string imgDir, StrList& leftImgPathList, StrList& rightImgPathList);

	static bool loadHandHeldImageList(std::string imgDir, FilePathMap& leftImgPathMap, FilePathMap& rightImgPathMap);

	static bool loadHandHeldImage1List(std::string imgDir, FilePathMap& leftImg1PathList, FilePathMap& rightImg1PathList);

	static bool loadHandHeldImage2List(std::string imgDir, FilePathMap& leftImg2PathList, FilePathMap& rightImg2PathList);

	static bool saveTrackImg(std::string outDir, Tracker* tracker);

	static bool saveMatchedMarkImg(std::string fileDir, Frame* frame);

private:
	static FILE* efopen(const char *file, char *mode);

private:

	static FILE*				_INFILE;
	static FILE*				_DFILE;
};


}


#endif // READWRITER_H